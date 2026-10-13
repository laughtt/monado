// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air HMD: 3DoF from the libpimax IMU stream.
 *
 * Based on the sample driver. Startup: the gyro is averaged for PIMAX_BIAS_SECONDS (default 2 s) while the
 * headset is still (window restarts on motion), the mean is used as gyro bias (the factory wBias is wrong,
 * see analysis/imu-orientation.md) and the initial tilt is seeded from the mean accelerometer.
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

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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
DEBUG_GET_ONCE_NUM_OPTION(pimax_bias_ms, "PIMAX_BIAS_MS", 2000)

#define PIMAX_EYE_W 3552
#define PIMAX_EYE_H 3840
#define PIMAX_REFRESH_HZ 72.0f
#define PIMAX_PUSH_DIV 4 // push every 4th IMU sample (250 Hz) into the relation history

struct pimax_hmd
{
	struct xrt_device base;
	enum u_logging_level log_level;

	pimax_t *lib;
	struct m_relation_history *relation_hist;
	struct m_imu_3dof fusion;

	int axis_mode;

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

	uint32_t count;
	float bias_dps[3];
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
	return t;
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
bias_finish(struct pimax_hmd *hmd)
{
	double n = hmd->bias_n;
	float g[3], a[3];
	for (int i = 0; i < 3; i++) {
		g[i] = (float)(hmd->gyro_sum[i] / n);
		a[i] = (float)(hmd->acc_sum[i] / n);
		hmd->bias_dps[i] = g[i];
	}
	struct xrt_vec3 gb, ac;
	map_axes(hmd, g, &gb);
	map_axes(hmd, a, &ac);

	hmd->fusion.gyro_bias.value = m_vec3_mul_scalar(gb, (float)(M_PI / 180.0));

	// Seed tilt: specific force points up, rotate it onto +Y.
	struct xrt_vec3 up = {0, 1, 0};
	struct xrt_vec3 ac_n = m_vec3_normalize(ac);
	math_quat_from_vec_a_to_vec_b(&ac_n, &up, &hmd->fusion.rot);
	math_quat_normalize(&hmd->fusion.rot);

	hmd->bias_done = true;
	HMD_INFO(hmd, "gyro bias done: %.3f %.3f %.3f dps over %u samples (%u restarts); accel %.3f %.3f %.3f g",
	         g[0], g[1], g[2], hmd->bias_n, hmd->restarts, a[0], a[1], a[2]);
}

//! Runs on the libpimax session thread.
static void
pimax_on_imu(void *user, const pimax_imu_sample *s)
{
	struct pimax_hmd *hmd = (struct pimax_hmd *)user;
	uint64_t ts_ns = sample_time_ns(hmd, s->ts_us);

	if (!hmd->bias_done) {
		// Reject the window when the headset is moving.
		double gm = fabs(s->gyro_dps[0]) + fabs(s->gyro_dps[1]) + fabs(s->gyro_dps[2]);
		if (gm > 8.0 + 3.0 * 1.0) { // bias itself is ~1 dps, so allow ~11 dps sum
			if (hmd->bias_n > 0) {
				hmd->restarts++;
			}
			bias_restart(hmd);
			return;
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
		if (hmd->bias_n >= hmd->bias_target) {
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
			} else {
				bias_finish(hmd);
			}
		}
		return;
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

	// Stop the session thread (and with it the IMU callback) before freeing anything.
	if (hmd->lib != NULL) {
		pimax_close(hmd->lib);
		hmd->lib = NULL;
	}

	u_var_remove_root(hmd);
	m_imu_3dof_close(&hmd->fusion);
	m_relation_history_destroy(&hmd->relation_hist);
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
	struct xrt_fov fov = xdev->hmd->distortion.fov[view_index];
	u_visibility_mask_get_default(type, &fov, out_mask);
	return XRT_SUCCESS;
}

struct xrt_device *
pimax_hmd_create(void)
{
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	struct pimax_hmd *hmd = U_DEVICE_ALLOCATE(struct pimax_hmd, flags, 1, 0);
	if (hmd == NULL) {
		return NULL;
	}

	hmd->log_level = debug_get_log_option_pimax_log();
	hmd->axis_mode = (int)debug_get_num_option_pimax_axis_mode();
	long bias_ms = debug_get_num_option_pimax_bias_ms();
	hmd->bias_target = (uint32_t)(bias_ms < 200 ? 200 : bias_ms); // 1 kHz IMU: ms == samples

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
	hmd->base.hmd->screens[0].nominal_frame_interval_ns = time_s_to_ns(1.0f / PIMAX_REFRESH_HZ);
	long div = debug_get_num_option_pimax_debug_div();
	if (div < 1) {
		div = 1;
	}
	const uint32_t eye_w = PIMAX_EYE_W / div, eye_h = PIMAX_EYE_H / div;
	hmd->base.hmd->screens[0].w_pixels = eye_w * 2;
	hmd->base.hmd->screens[0].h_pixels = eye_h;

	// Simple symmetric FOV, no lens data yet (doc: ~98 x 87 deg max).
	const double hFOV = 98.0 * (M_PI / 180.0);
	const double vFOV = 87.0 * (M_PI / 180.0);
	if (!math_compute_fovs(1, 0.5, hFOV, 1, 0.5, vFOV, &hmd->base.hmd->distortion.fov[0]) ||
	    !math_compute_fovs(1, 0.5, hFOV, 1, 0.5, vFOV, &hmd->base.hmd->distortion.fov[1])) {
		HMD_ERROR(hmd, "Failed to compute FOVs");
		u_device_free(&hmd->base);
		return NULL;
	}

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

	// TODO: real lens distortion (Dream Air tables are GPL, must be re-measured).
	u_distortion_mesh_set_none(&hmd->base);

	m_relation_history_create(&hmd->relation_hist);
	m_imu_3dof_init(&hmd->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

	// Initial identity so queries before the bias window finishes have something sane (not tracked).
	struct xrt_space_relation identity = XRT_SPACE_RELATION_ZERO;
	identity.relation_flags = XRT_SPACE_RELATION_ORIENTATION_VALID_BIT;
	m_relation_history_push(hmd->relation_hist, &identity, os_monotonic_get_ns());

	bias_restart(hmd);

	pimax_callbacks cb = {.user = hmd, .on_imu = pimax_on_imu};
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
	int prep = pimax_video_prepare(hmd->lib, 72);
	HMD_INFO(hmd, "pimax_video_prepare(72) -> %d", prep);
	const char *xdg = getenv("XDG_RUNTIME_DIR");
	if (xdg != NULL) {
		char path[512];
		snprintf(path, sizeof(path), "%s/pimax-rate", xdg);
		FILE *f = fopen(path, "w");
		if (f != NULL) {
			fputs("72\n", f);
			fclose(f);
			HMD_INFO(hmd, "wrote 72 to %s", path);
		} else {
			HMD_WARN(hmd, "could not write %s", path);
		}
	}

	u_var_add_root(hmd, "Pimax Dream Air", true);
	u_var_add_log_level(hmd, &hmd->log_level, "log_level");
	u_var_add_bool(hmd, &hmd->bias_done, "gyro bias done");
	u_var_add_f32(hmd, &hmd->bias_dps[0], "bias x dps");
	u_var_add_f32(hmd, &hmd->bias_dps[1], "bias y dps");
	u_var_add_f32(hmd, &hmd->bias_dps[2], "bias z dps");

	HMD_INFO(hmd, "Pimax Dream Air up: keep the headset still for %u ms for gyro calibration (axis mode %d)",
	         hmd->bias_target, hmd->axis_mode);
	return &hmd->base;
}
