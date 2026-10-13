// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air HMD: 3DoF from the libpimax IMU stream.
 *
 * Based on the sample driver. Startup: the gyro is averaged for PIMAX_BIAS_SECONDS (default 2 s) while the
 * headset is still (window restarts on motion), the mean is used as gyro bias (the factory wBias is wrong,
 * see analysis/imu-orientation.md) and the initial tilt is seeded from the mean accelerometer.
 * If no still window comes within PIMAX_BIAS_TIMEOUT_MS (default 3000, e.g. started while worn), tracking starts
 * with the bias cached by the last good calibration (~/.cache/pimax/gyro-bias, else zero) and the window keeps
 * running: every later still window replaces the bias.
 *
 * @ingroup drv_pimax
 */

#include "os/os_time.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#include "math/m_relation_history.h"
#include "math/m_imu_3dof.h"
#include "math/m_api.h"
#include "math/m_vec3.h"
#include "math/m_matrix_2x2.h"
#include "math/m_mathinclude.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_distortion_mesh.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_var.h"
#include "util/u_visibility_mask.h"

#include "pimax.h"
#include "pimax_interface.h"
#include "pimax_lens.h"
#include "pimax_ctrl.h"
#include "pimax_camera.h"

#include "os/os_threading.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

DEBUG_GET_ONCE_LOG_OPTION(pimax_log, "PIMAX_LOG", U_LOGGING_INFO)
//! TODO: the panels are mounted rotated by 90 deg (direction unknown). 0, 90, 180 or 270.
DEBUG_GET_ONCE_NUM_OPTION(pimax_panel_rotation, "PIMAX_PANEL_ROTATION", 0) // legacy: both eyes
DEBUG_GET_ONCE_NUM_OPTION(pimax_rot_left, "PIMAX_ROT_LEFT", 90)
DEBUG_GET_ONCE_NUM_OPTION(pimax_rot_right, "PIMAX_ROT_RIGHT", 270)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_mirror_left, "PIMAX_MIRROR_LEFT", false)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_mirror_right, "PIMAX_MIRROR_RIGHT", false)
//! 0: sensor axes == OpenXR axes (+X right, +Y up, -Z fwd). 1: 180 deg about Y (X and Z flipped).
//! Debug only: divide screen/view sizes so the windowed (XCB/Wayland) window fits a normal monitor.
DEBUG_GET_ONCE_NUM_OPTION(pimax_debug_div, "PIMAX_DEBUG_DIV", 1)
DEBUG_GET_ONCE_NUM_OPTION(pimax_axis_mode, "PIMAX_AXIS_MODE", 1)
//! Debug: do not open the headset (no USB access, no tracking): lens/compositor tests without hardware.
DEBUG_GET_ONCE_BOOL_OPTION(pimax_no_device, "PIMAX_NO_DEVICE", false)
DEBUG_GET_ONCE_NUM_OPTION(pimax_bias_ms, "PIMAX_BIAS_MS", 2000)
DEBUG_GET_ONCE_NUM_OPTION(pimax_bias_timeout_ms, "PIMAX_BIAS_TIMEOUT_MS", 3000)

#define PIMAX_EYE_W 3552
#define PIMAX_EYE_H 3840
//! Refresh rate: 72 (default) or 90. Must match the mode Monado picks (XRT_COMPOSITOR_DESIRED_MODE).
DEBUG_GET_ONCE_NUM_OPTION(pimax_fps, "PIMAX_FPS", 72)
#define PIMAX_PUSH_DIV 4 // push every 4th IMU sample (250 Hz) into the relation history

struct pimax_hmd
{
	struct xrt_device base;
	enum u_logging_level log_level;

	pimax_t *lib;
	struct m_relation_history *relation_hist;
	struct m_imu_3dof fusion;

	int axis_mode;
	struct pimax_lens lens;

	struct pimax_ctrl_ctx *ctrl;
	struct xrt_device *ctrl_devs[2];
	bool ctrl_handed_out;

	// Device clock -> monotonic clock.
	bool have_clock;
	uint64_t dev_ts_ext_us; // unwrapped device time
	uint32_t last_dev_ts;
	int64_t clock_offset_ns;
	uint64_t last_ts_ns;

	// Startup bias estimation.
	bool bias_done;
	uint32_t bias_n;
	uint32_t bias_target;
	double gyro_sum[3];
	double acc_sum[3];
	double half_sum[3]; // sum over first half of the window
	uint32_t restarts;
	uint32_t total_n;   // samples since start, for the timeout
	uint32_t timeout_n; // 1 kHz IMU: ms == samples
	bool bias_fallback; // tracking started with the cached bias; still windows keep refining it

	uint32_t count;
	float bias_dps[3];

	//! IMU clock snapshot for the cameras (written by the IMU callback).
	struct os_mutex clk_lock;
	struct pimax_camera_clock_ref clk;
	//! Tracking cameras + Mercury (NULL unless PIMAX_HANDS / PIMAX_CAM).
	struct pimax_camera *camera;
};

static inline struct pimax_hmd *
pimax_hmd(struct xrt_device *xdev)
{
	return (struct pimax_hmd *)xdev;
}

#define HMD_INFO(hmd, ...) U_LOG_XDEV_IFL_I(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_WARN(hmd, ...) U_LOG_XDEV_IFL_W(&hmd->base, hmd->log_level, __VA_ARGS__)
#define HMD_ERROR(hmd, ...) U_LOG_XDEV_IFL_E(&hmd->base, hmd->log_level, __VA_ARGS__)

static void
map_axes(const struct pimax_hmd *hmd, const float in[3], struct xrt_vec3 *out)
{
	if (hmd->axis_mode == 1) {
		out->x = -in[0];
		out->y = in[1];
		out->z = -in[2];
	} else {
		out->x = in[0];
		out->y = in[1];
		out->z = in[2];
	}
}

static uint64_t
sample_time_ns(struct pimax_hmd *hmd, uint32_t dev_ts_us)
{
	uint64_t now = os_monotonic_get_ns();
	if (!hmd->have_clock) {
		hmd->have_clock = true;
		hmd->dev_ts_ext_us = dev_ts_us;
		hmd->last_dev_ts = dev_ts_us;
		hmd->clock_offset_ns = (int64_t)now - (int64_t)(hmd->dev_ts_ext_us * 1000);
	} else {
		hmd->dev_ts_ext_us += (uint32_t)(dev_ts_us - hmd->last_dev_ts); // handles 2^32 wrap
		hmd->last_dev_ts = dev_ts_us;
	}
	int64_t cand = (int64_t)now - (int64_t)(hmd->dev_ts_ext_us * 1000);
	// Track the minimum (least delayed) offset; relax slowly to follow clock drift.
	if (cand < hmd->clock_offset_ns) {
		hmd->clock_offset_ns = cand;
	} else {
		hmd->clock_offset_ns += (cand - hmd->clock_offset_ns) >> 14;
	}
	uint64_t t = (uint64_t)((int64_t)(hmd->dev_ts_ext_us * 1000) + hmd->clock_offset_ns);
	if (t <= hmd->last_ts_ns) {
		t = hmd->last_ts_ns + 1;
	}
	hmd->last_ts_ns = t;

	os_mutex_lock(&hmd->clk_lock);
	hmd->clk.dev_ts_u32 = dev_ts_us;
	hmd->clk.dev_ext_us = hmd->dev_ts_ext_us;
	hmd->clk.offset_ns = hmd->clock_offset_ns;
	hmd->clk.valid = 1;
	os_mutex_unlock(&hmd->clk_lock);
	return t;
}

//! Camera thread: the IMU device-clock -> monotonic mapping, so frames share the IMU timeline.
static bool
pimax_hmd_clock_ref(void *user, struct pimax_camera_clock_ref *out)
{
	struct pimax_hmd *hmd = (struct pimax_hmd *)user;
	os_mutex_lock(&hmd->clk_lock);
	*out = hmd->clk;
	os_mutex_unlock(&hmd->clk_lock);
	return out->valid != 0;
}

struct xrt_device *
pimax_hmd_take_hand_tracker(struct xrt_device *xdev)
{
	return pimax_camera_take_hand_device(pimax_hmd(xdev)->camera);
}

static void
bias_restart(struct pimax_hmd *hmd)
{
	hmd->bias_n = 0;
	for (int i = 0; i < 3; i++) {
		hmd->gyro_sum[i] = hmd->acc_sum[i] = hmd->half_sum[i] = 0;
	}
}

static void
bias_cache_path(char *out, size_t n)
{
	const char *xdg = getenv("XDG_CACHE_HOME");
	const char *home = getenv("HOME");
	if (xdg != NULL && xdg[0] != '\0') {
		snprintf(out, n, "%s/pimax/gyro-bias", xdg);
	} else {
		snprintf(out, n, "%s/.cache/pimax/gyro-bias", home != NULL ? home : "/tmp");
	}
}

static bool
bias_cache_load(float g[3])
{
	char path[512];
	bias_cache_path(path, sizeof(path));
	FILE *f = fopen(path, "r");
	if (f == NULL) {
		return false;
	}
	bool ok = fscanf(f, "%f %f %f", &g[0], &g[1], &g[2]) == 3;
	fclose(f);
	// A sane bias is ~1 dps; anything large is a bad file.
	for (int i = 0; ok && i < 3; i++) {
		ok = isfinite(g[i]) && fabsf(g[i]) < 5.0f;
	}
	return ok;
}

static void
bias_cache_save(const float g[3])
{
	char path[512];
	bias_cache_path(path, sizeof(path));
	char *slash = strrchr(path, '/');
	if (slash != NULL) {
		*slash = '\0';
		mkdir(path, 0755); // parent ~/.cache exists on any desktop
		*slash = '/';
	}
	FILE *f = fopen(path, "w");
	if (f != NULL) {
		fprintf(f, "%.4f %.4f %.4f\n", g[0], g[1], g[2]);
		fclose(f);
	}
}

static void
bias_set(struct pimax_hmd *hmd, const float g[3])
{
	struct xrt_vec3 gb;
	map_axes(hmd, g, &gb);
	hmd->fusion.gyro_bias.value = m_vec3_mul_scalar(gb, (float)(M_PI / 180.0));
	for (int i = 0; i < 3; i++) {
		hmd->bias_dps[i] = g[i];
	}
}

static void
seed_tilt(struct pimax_hmd *hmd, const float a[3])
{
	// Specific force points up, rotate it onto +Y.
	struct xrt_vec3 ac, up = {0, 1, 0};
	map_axes(hmd, a, &ac);
	struct xrt_vec3 ac_n = m_vec3_normalize(ac);
	math_quat_from_vec_a_to_vec_b(&ac_n, &up, &hmd->fusion.rot);
	math_quat_normalize(&hmd->fusion.rot);
}

//! A still window completed: first time it also seeds the tilt and starts tracking; later it only updates the bias.
static void
bias_finish(struct pimax_hmd *hmd)
{
	double n = hmd->bias_n;
	float g[3], a[3];
	for (int i = 0; i < 3; i++) {
		g[i] = (float)(hmd->gyro_sum[i] / n);
		a[i] = (float)(hmd->acc_sum[i] / n);
	}
	bias_set(hmd, g);
	bias_cache_save(g);

	if (hmd->bias_fallback) {
		hmd->bias_fallback = false;
		HMD_INFO(hmd, "gyro bias refined while running: %.3f %.3f %.3f dps", g[0], g[1], g[2]);
		return;
	}
	seed_tilt(hmd, a);
	hmd->bias_done = true;
	HMD_INFO(hmd, "gyro bias done: %.3f %.3f %.3f dps over %u samples (%u restarts); accel %.3f %.3f %.3f g",
	         g[0], g[1], g[2], hmd->bias_n, hmd->restarts, a[0], a[1], a[2]);
}

//! No still window in time (headset worn at start): track now with the cached bias.
static void
bias_timeout(struct pimax_hmd *hmd, const float accel_g[3])
{
	float g[3] = {0, 0, 0};
	bool cached = bias_cache_load(g);
	bias_set(hmd, g);
	seed_tilt(hmd, accel_g);
	hmd->bias_fallback = true;
	hmd->bias_done = true;
	bias_restart(hmd);
	HMD_WARN(hmd,
	         "headset not still for %u ms (%u restarts): tracking with the %s gyro bias %.3f %.3f %.3f dps; "
	         "it is refined the next time the headset rests for %u ms",
	         hmd->timeout_n, hmd->restarts, cached ? "cached" : "zero (no cache)", g[0], g[1], g[2],
	         hmd->bias_target);
}

//! Feed one sample into the still window. Returns true when a window completed (bias_finish was called).
static bool
bias_feed(struct pimax_hmd *hmd, const pimax_imu_sample *s)
{
	// Reject the window when the headset is moving.
	double gm = fabs(s->gyro_dps[0]) + fabs(s->gyro_dps[1]) + fabs(s->gyro_dps[2]);
	if (gm > 8.0 + 3.0 * 1.0) { // bias itself is ~1 dps, so allow ~11 dps sum
		if (hmd->bias_n > 0) {
			hmd->restarts++;
		}
		bias_restart(hmd);
		return false;
	}
	for (int i = 0; i < 3; i++) {
		hmd->gyro_sum[i] += s->gyro_dps[i];
		hmd->acc_sum[i] += s->accel_g[i];
	}
	hmd->bias_n++;
	if (hmd->bias_n == hmd->bias_target / 2) {
		for (int i = 0; i < 3; i++) {
			hmd->half_sum[i] = hmd->gyro_sum[i];
		}
	}
	if (hmd->bias_n < hmd->bias_target) {
		return false;
	}
	// First half vs second half mean must agree (< 0.3 dps) else it was not still.
	double h = hmd->bias_target / 2.0;
	double dmax = 0;
	for (int i = 0; i < 3; i++) {
		double m1 = hmd->half_sum[i] / h;
		double m2 = (hmd->gyro_sum[i] - hmd->half_sum[i]) / (hmd->bias_n - h);
		dmax = fmax(dmax, fabs(m1 - m2));
	}
	if (dmax > 0.3) {
		hmd->restarts++;
		bias_restart(hmd);
		return false;
	}
	bias_finish(hmd);
	return true;
}

//! Runs on the libpimax session thread.
static void
pimax_on_controller(void *user, const pimax_controller_report *r)
{
	pimax_ctrl_on_report(((struct pimax_hmd *)user)->ctrl, r);
}

static void
pimax_on_controller_status(void *user, const pimax_controller_status *s)
{
	pimax_ctrl_on_status(((struct pimax_hmd *)user)->ctrl, s);
}

static void
pimax_on_state(void *user, pimax_state state)
{
	pimax_ctrl_on_state(((struct pimax_hmd *)user)->ctrl, state);
}

static void
pimax_on_imu(void *user, const pimax_imu_sample *s)
{
	struct pimax_hmd *hmd = (struct pimax_hmd *)user;
	uint64_t ts_ns = sample_time_ns(hmd, s->ts_us);

	if (!hmd->bias_done) {
		if (!bias_feed(hmd, s) && ++hmd->total_n >= hmd->timeout_n) {
			bias_timeout(hmd, s->accel_g);
		}
		return;
	}
	if (hmd->bias_fallback) {
		bias_feed(hmd, s);
	}

	float a[3] = {s->accel_g[0] * 9.80665f, s->accel_g[1] * 9.80665f, s->accel_g[2] * 9.80665f};
	float g[3] = {s->gyro_dps[0] * (float)(M_PI / 180.0), s->gyro_dps[1] * (float)(M_PI / 180.0),
	              s->gyro_dps[2] * (float)(M_PI / 180.0)};
	struct xrt_vec3 accel, gyro;
	map_axes(hmd, a, &accel);
	map_axes(hmd, g, &gyro);

	m_imu_3dof_update(&hmd->fusion, ts_ns, &accel, &gyro);

	if ((++hmd->count % PIMAX_PUSH_DIV) != 0) {
		return;
	}

	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                     XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	                                                     XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT);
	rel.pose.orientation = hmd->fusion.rot;
	// Angular velocity in world space (bias corrected) so the history can predict.
	struct xrt_vec3 gb = m_vec3_sub(gyro, hmd->fusion.gyro_bias.value);
	math_quat_rotate_vec3(&hmd->fusion.rot, &gb, &rel.angular_velocity);
	m_relation_history_push(hmd->relation_hist, &rel, ts_ns);
}

static void
pimax_hmd_destroy(struct xrt_device *xdev)
{
	struct pimax_hmd *hmd = pimax_hmd(xdev);

	// Cameras first: their thread reads the IMU clock snapshot and pushes into Mercury.
	pimax_camera_destroy(&hmd->camera);

	// Stop the controller attach thread, then the session thread (and with it the callbacks) before freeing anything.
	if (hmd->ctrl != NULL) {
		pimax_ctrl_stop(hmd->ctrl);
		if (!hmd->ctrl_handed_out) {
			// Creation failed after the controllers were made: nobody else will destroy them.
			xrt_device_destroy(&hmd->ctrl_devs[0]);
			xrt_device_destroy(&hmd->ctrl_devs[1]);
		}
	}
	if (hmd->lib != NULL) {
		pimax_close(hmd->lib);
		hmd->lib = NULL;
	}

	u_var_remove_root(hmd);
	pimax_lens_destroy(&hmd->lens);
	m_imu_3dof_close(&hmd->fusion);
	m_relation_history_destroy(&hmd->relation_hist);
	os_mutex_destroy(&hmd->clk_lock);
	u_device_free(&hmd->base);
}

static xrt_result_t
pimax_hmd_update_inputs(struct xrt_device *xdev)
{
	return XRT_SUCCESS;
}

static xrt_result_t
pimax_hmd_get_tracked_pose(struct xrt_device *xdev,
                           enum xrt_input_name name,
                           int64_t at_timestamp_ns,
                           struct xrt_space_relation *out_relation)
{
	struct pimax_hmd *hmd = pimax_hmd(xdev);

	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&hmd->base, hmd->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_space_relation relation = XRT_SPACE_RELATION_ZERO;
	m_relation_history_get(hmd->relation_hist, at_timestamp_ns, &relation);

	if ((relation.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) != 0) {
		math_quat_normalize(&relation.pose.orientation);
	}

	*out_relation = relation;
	return XRT_SUCCESS;
}

static xrt_result_t
pimax_hmd_get_view_poses(struct xrt_device *xdev,
                         const struct xrt_vec3 *default_eye_relation,
                         int64_t at_timestamp_ns,
                         enum xrt_view_type view_type,
                         uint32_t view_count,
                         struct xrt_space_relation *out_head_relation,
                         struct xrt_fov *out_fovs,
                         struct xrt_pose *out_poses)
{
	return u_device_get_view_poses(xdev, default_eye_relation, at_timestamp_ns, view_type, view_count,
	                               out_head_relation, out_fovs, out_poses);
}

static xrt_result_t
pimax_hmd_get_visibility_mask(struct xrt_device *xdev,
                              enum xrt_visibility_mask_type type,
                              uint32_t view_index,
                              struct xrt_visibility_mask **out_mask)
{
	struct pimax_hmd *hmd = pimax_hmd(xdev);
	// A configured edge black-out / mask (lens.conf) gets its own mesh, otherwise the generic FOV mask.
	struct xrt_visibility_mask *custom = pimax_lens_visibility_mask(&hmd->lens, type, view_index);
	if (custom != NULL) {
		*out_mask = custom;
		return XRT_SUCCESS;
	}
	struct xrt_fov fov = xdev->hmd->distortion.fov[view_index];
	u_visibility_mask_get_default(type, &fov, out_mask);
	return XRT_SUCCESS;
}

static xrt_result_t
pimax_hmd_compute_distortion(struct xrt_device *xdev, uint32_t view, float u, float v, struct xrt_uv_triplet *result)
{
	struct pimax_hmd *hmd = (struct pimax_hmd *)xdev;
	return pimax_lens_compute(&hmd->lens, view, u, v, result);
}

struct xrt_device *
pimax_hmd_create(struct xrt_device *out_ctrls[2])
{
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	struct pimax_hmd *hmd = U_DEVICE_ALLOCATE(struct pimax_hmd, flags, 1, 0);
	if (hmd == NULL) {
		return NULL;
	}

	hmd->log_level = debug_get_log_option_pimax_log();
	os_mutex_init(&hmd->clk_lock);
	hmd->axis_mode = (int)debug_get_num_option_pimax_axis_mode();
	long bias_ms = debug_get_num_option_pimax_bias_ms();
	hmd->bias_target = (uint32_t)(bias_ms < 200 ? 200 : bias_ms); // 1 kHz IMU: ms == samples
	long timeout_ms = debug_get_num_option_pimax_bias_timeout_ms();
	hmd->timeout_n = (uint32_t)(timeout_ms < 200 ? 200 : timeout_ms);

	hmd->base.update_inputs = pimax_hmd_update_inputs;
	hmd->base.get_tracked_pose = pimax_hmd_get_tracked_pose;
	hmd->base.get_view_poses = pimax_hmd_get_view_poses;
	hmd->base.get_visibility_mask = pimax_hmd_get_visibility_mask;
	hmd->base.destroy = pimax_hmd_destroy;

	snprintf(hmd->base.str, XRT_DEVICE_NAME_LEN, "Pimax Dream Air");
	snprintf(hmd->base.serial, XRT_DEVICE_NAME_LEN, "Pimax Dream Air");

	hmd->base.name = XRT_DEVICE_GENERIC_HMD;
	hmd->base.device_type = XRT_DEVICE_TYPE_HMD;
	hmd->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;
	hmd->base.supported.orientation_tracking = true;
	hmd->base.supported.position_tracking = false;

	hmd->base.hmd->blend_modes[0] = XRT_BLEND_MODE_OPAQUE;
	hmd->base.hmd->blend_mode_count = 1;

	// Display: one 7104x3840 @ 72 Hz output, left half = left eye, right half = right eye.
	hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / (float)debug_get_num_option_pimax_fps());
	long div = debug_get_num_option_pimax_debug_div();
	if (div < 1) {
		div = 1;
	}
	const uint32_t eye_w = PIMAX_EYE_W / div, eye_h = PIMAX_EYE_H / div;
	hmd->base.hmd->screens[0].w_pixels = eye_w * 2;
	hmd->base.hmd->screens[0].h_pixels = eye_h;

	// Lens model + FOV from ~/.config/pimax/lens.conf (or PIMAX_LENS_CONF), see analysis/lens-distortion.md.
	pimax_lens_load(&hmd->lens, (float)PIMAX_EYE_H / (float)PIMAX_EYE_W);
	hmd->base.hmd->distortion.fov[0] = pimax_lens_get_fov(&hmd->lens, 0);
	hmd->base.hmd->distortion.fov[1] = pimax_lens_get_fov(&hmd->lens, 1);

	/*
	 * Panels are mounted rotated by 90 deg and mirror-symmetric: the left panel shows its half of the frame
	 * rotated 90 deg counter-clockwise, the right panel 90 deg clockwise. The compositor pre-rotates each eye the
	 * opposite way (u_device_rotation_*: "right" = 90 cw, "left" = 90 ccw as applied to the image).
	 * PIMAX_ROT_LEFT / PIMAX_ROT_RIGHT select 0/90/180/270 (degrees of pre-rotation, clockwise), optional
	 * PIMAX_MIRROR_LEFT / PIMAX_MIRROR_RIGHT flip the pre-rotated image horizontally.
	 * With 90/270 the user-facing display is eye_h x eye_w (3840x3552) while the framebuffer viewport stays
	 * eye_w x eye_h (3552x3840).
	 */
	long rots[2] = {debug_get_num_option_pimax_rot_left(), debug_get_num_option_pimax_rot_right()};
	long legacy = debug_get_num_option_pimax_panel_rotation();
	if (legacy != 0) {
		rots[0] = rots[1] = legacy;
	}
	bool mirror[2] = {debug_get_bool_option_pimax_mirror_left(), debug_get_bool_option_pimax_mirror_right()};
	for (uint8_t eye = 0; eye < 2; ++eye) {
		const struct xrt_matrix_2x2 *rotm = &u_device_rotation_ident;
		long r = ((rots[eye] % 360) + 360) % 360;
		switch (r) {
		case 90: rotm = &u_device_rotation_right; break;
		case 180: rotm = &u_device_rotation_180; break;
		case 270: rotm = &u_device_rotation_left; break;
		default: r = 0; break;
		}
		bool swap = (r == 90 || r == 270);
		hmd->base.hmd->views[eye].display.w_pixels = swap ? eye_h : eye_w;
		hmd->base.hmd->views[eye].display.h_pixels = swap ? eye_w : eye_h;
		hmd->base.hmd->views[eye].viewport.x_pixels = eye * eye_w;
		hmd->base.hmd->views[eye].viewport.y_pixels = 0;
		hmd->base.hmd->views[eye].viewport.w_pixels = eye_w;
		hmd->base.hmd->views[eye].viewport.h_pixels = eye_h;
		struct xrt_matrix_2x2 m = *rotm;
		if (mirror[eye]) {
			const struct xrt_matrix_2x2 flip = {{.vecs = {{-1, 0}, {0, 1}}}};
			m_mat2x2_multiply(&m, &flip, &m);
		}
		hmd->base.hmd->views[eye].rot = m;
		HMD_INFO(hmd, "eye %u: pre-rotate %ld deg%s, display %ux%u, viewport %ux%u", eye, r,
		         mirror[eye] ? " + mirror" : "", hmd->base.hmd->views[eye].display.w_pixels,
		         hmd->base.hmd->views[eye].display.h_pixels, eye_w, eye_h);
	}

	// Distortion mesh from our own lens model (u, v arrive already in the pre-rotated, user-facing display space).
	hmd->base.compute_distortion = pimax_hmd_compute_distortion;
	hmd->base.hmd->distortion.models = XRT_DISTORTION_MODEL_COMPUTE;
	hmd->base.hmd->distortion.preferred = XRT_DISTORTION_MODEL_COMPUTE;
	u_distortion_mesh_fill_in_compute(&hmd->base);

	m_relation_history_create(&hmd->relation_hist);
	m_imu_3dof_init(&hmd->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

	// Initial identity so queries before the bias window finishes have something sane (not tracked).
	struct xrt_space_relation identity = XRT_SPACE_RELATION_ZERO;
	identity.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	m_relation_history_push(hmd->relation_hist, &identity, os_monotonic_get_ns());

	if (debug_get_bool_option_pimax_no_device()) {
		HMD_WARN(hmd, "PIMAX_NO_DEVICE: headset not opened, identity pose");
		u_var_add_root(hmd, "Pimax Dream Air (no device)", true);
		// Test aid for the controller hand tracking: with PIMAX_CTRL_HANDS_DEMO=1 the controllers exist even without
		// a headset (no attach thread, no reports; the scripted demo input drives the hands).
		if (pimax_ctrl_demo_enabled()) {
			hmd->ctrl = pimax_ctrl_create(&hmd->base, hmd->log_level, hmd->ctrl_devs);
			if (hmd->ctrl != NULL) {
				out_ctrls[0] = hmd->ctrl_devs[0];
				out_ctrls[1] = hmd->ctrl_devs[1];
				hmd->ctrl_handed_out = true;
			}
		}
		// Cameras still work (live UVC or a PIMAX_CAM_SOURCE replay), timed without the IMU clock.
		hmd->camera = pimax_camera_create(&hmd->base, hmd->log_level, NULL, NULL);
		return &hmd->base;
	}

	bias_restart(hmd);

	hmd->ctrl = pimax_ctrl_create(&hmd->base, hmd->log_level, hmd->ctrl_devs);
	if (hmd->ctrl == NULL) {
		HMD_WARN(hmd, "controllers not created");
	}

	pimax_callbacks cb = {.user = hmd,
	                      .on_imu = pimax_on_imu,
	                      .on_state = pimax_on_state,
	                      .on_controller = pimax_on_controller,
	                      .on_controller_status = pimax_on_controller_status};
	hmd->lib = pimax_open(&cb);
	if (hmd->lib == NULL) {
		HMD_ERROR(hmd, "pimax_open failed");
		pimax_hmd_destroy(&hmd->base);
		return NULL;
	}
	if (pimax_wait_connected(hmd->lib, 5000) != 0) {
		HMD_ERROR(hmd, "Pimax headset did not connect within 5 s");
		pimax_hmd_destroy(&hmd->base);
		return NULL;
	}

	// Firmware only lights the panels after the video-prepare sequence and ~5.5 s of stable 72 Hz video.
	// Also tell the keepalive service (pimax-keepalived) which rate to carry.
	long fps = debug_get_num_option_pimax_fps();
	int prep = pimax_video_prepare(hmd->lib, (uint8_t)fps);
	HMD_INFO(hmd, "pimax_video_prepare(%ld) -> %d", fps, prep);
	const char *xdg = getenv("XDG_RUNTIME_DIR");
	if (xdg != NULL) {
		char path[512];
		snprintf(path, sizeof(path), "%s/pimax-rate", xdg);
		FILE *f = fopen(path, "w");
		if (f != NULL) {
			fprintf(f, "%ld\n", fps);
			fclose(f);
			HMD_INFO(hmd, "wrote %ld to %s", fps, path);
		} else {
			HMD_WARN(hmd, "could not write %s", path);
		}
	}

	if (hmd->ctrl != NULL) {
		pimax_ctrl_start(hmd->ctrl, hmd->lib);
		out_ctrls[0] = hmd->ctrl_devs[0];
		out_ctrls[1] = hmd->ctrl_devs[1];
		hmd->ctrl_handed_out = true;
	}

	u_var_add_root(hmd, "Pimax Dream Air", true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");
	u_var_add_bool(hmd, &hmd->bias_done, "gyro bias done");
	u_var_add_f32(hmd, &hmd->bias_dps[0], "bias x dps");
	u_var_add_f32(hmd, &hmd->bias_dps[1], "bias y dps");
	u_var_add_f32(hmd, &hmd->bias_dps[2], "bias z dps");

	// Tracking cameras and Mercury hands (PIMAX_HANDS=mercury or PIMAX_CAM=1); NULL otherwise.
	hmd->camera = pimax_camera_create(&hmd->base, hmd->log_level, pimax_hmd_clock_ref, hmd);

	HMD_INFO(hmd, "Pimax Dream Air up: keep the headset still for %u ms for gyro calibration (axis mode %d)",
	         hmd->bias_target, hmd->axis_mode);
	return &hmd->base;
}
