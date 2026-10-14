// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air tracking cameras -> xrt_frame sinks -> Mercury. Thin glue: capture, tile split,
 *         metadata, timestamps and calibration parsing live in the Rust crate pimax-cam (pimax_cam.h).
 *
 * Graph (one capture thread):
 *   pimax-cam (V4L2 or .pxcam replay) -> tracking-exposure frames (25 Hz; LED frames only reach the debug sinks)
 *     view 0 / view 1 -> u_sink_split -> Mercury view sink (t_hand_tracking_async)
 *                                     -> SLAM placeholder sink (counts frames)
 *     PIMAX_SLAM_CAMS tiles (IMU-timed only) -> SLAM tracker (t_slam / VIT, Basalt) and/or EuRoC recorder
 *   HMD IMU thread -> pimax_camera_push_imu -> SLAM tracker IMU sink and/or EuRoC recorder
 *   Mercury device -> multi tracking override attached to the HMD head pose -> XR_EXT_hand_tracking.
 *   HMD get_tracked_pose -> pimax_camera_head_relation: tracker pose checked by the Rust supervisor
 *     (pimax_cam_track_*: acquire/lose/reset, continuity) or the 3DoF orientation at the held position.
 *
 * @ingroup drv_pimax
 */

#include "pimax_camera.h"

#ifndef PIMAX_HAVE_CAM

// Built without -DPIMAX_CAM=ON: no camera support, the HMD works as before.
#include "util/u_debug.h"
#include "util/u_logging.h"

DEBUG_GET_ONCE_OPTION(pimax_hands_stub, "PIMAX_HANDS", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_cam_stub, "PIMAX_CAM", false)

DEBUG_GET_ONCE_BOOL_OPTION(pimax_slam_stub, "PIMAX_SLAM", false)
DEBUG_GET_ONCE_OPTION(pimax_slam_record_stub, "PIMAX_SLAM_RECORD", NULL)

struct pimax_camera *
pimax_camera_create(struct xrt_device *hmd,
                    enum u_logging_level log_level,
                    pimax_camera_clock_fn clock,
                    void *clock_user,
                    pimax_camera_orient_fn orient,
                    void *orient_user,
                    pimax_camera_send_fn send,
                    void *send_user)
{
	(void)hmd;
	(void)clock;
	(void)clock_user;
	(void)orient;
	(void)orient_user;
	(void)send;
	(void)send_user;
	const char *hands = debug_get_option_pimax_hands_stub();
	if (hands != NULL || debug_get_bool_option_pimax_cam_stub() || debug_get_bool_option_pimax_slam_stub() ||
	    debug_get_option_pimax_slam_record_stub() != NULL) {
		U_LOG_IFL_W(log_level,
		            "PIMAX_HANDS/PIMAX_CAM/PIMAX_SLAM set, but Monado was built without -DPIMAX_CAM=ON");
	}
	return NULL;
}

bool
pimax_camera_wants_imu(const struct pimax_camera *pc)
{
	(void)pc;
	return false;
}

void
pimax_camera_push_imu(struct pimax_camera *pc, int64_t ts_ns, const struct xrt_vec3 *accel_m_s2,
                      const struct xrt_vec3 *gyro_rad_s)
{
	(void)pc;
	(void)ts_ns;
	(void)accel_m_s2;
	(void)gyro_rad_s;
}

bool
pimax_camera_head_relation(struct pimax_camera *pc,
                           int64_t at_ns,
                           const struct xrt_space_relation *rel3dof,
                           struct xrt_space_relation *out)
{
	(void)pc;
	(void)at_ns;
	(void)rel3dof;
	(void)out;
	return false;
}

struct xrt_device *
pimax_camera_take_hand_device(struct pimax_camera *pc)
{
	(void)pc;
	return NULL;
}

void
pimax_camera_destroy(struct pimax_camera **pc_ptr)
{
	(void)pc_ptr;
}

#else // PIMAX_HAVE_CAM

#include "xrt/xrt_config_drivers.h"
#include "xrt/xrt_frame.h"
#include "xrt/xrt_tracking.h"

#include "os/os_threading.h"
#include "os/os_time.h"

#include "math/m_api.h"

#include "util/u_debug.h"
#include "util/u_frame.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_sink.h"
#include "util/u_trace_marker.h"
#include "util/u_var.h"

#include "tracking/t_tracking.h"
#include "tracking/t_hand_tracking.h"
#include "tracking/t_euroc_recorder.h"

#ifdef XRT_BUILD_DRIVER_HANDTRACKING
#include "../ht/ht_interface.h"
#include "../multi_wrapper/multi.h"
#endif

#include "pimax_cam.h"

#include <assert.h>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

DEBUG_GET_ONCE_OPTION(pimax_hands, "PIMAX_HANDS", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_cam, "PIMAX_CAM", false)
DEBUG_GET_ONCE_OPTION(pimax_cam_source, "PIMAX_CAM_SOURCE", NULL)
DEBUG_GET_ONCE_OPTION(pimax_cam_record, "PIMAX_CAM_RECORD", NULL)
DEBUG_GET_ONCE_OPTION(pimax_cam_calib, "PIMAX_CAM_CALIB", NULL)
DEBUG_GET_ONCE_OPTION(pimax_ht_cams, "PIMAX_HT_CAMS", "0,1")
DEBUG_GET_ONCE_OPTION(pimax_ht_orient, "PIMAX_HT_ORIENT", NULL)
DEBUG_GET_ONCE_OPTION(pimax_ht_head_offset, "PIMAX_HT_HEAD_OFFSET", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ht_stats, "PIMAX_HT_STATS", false)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_slam, "PIMAX_SLAM", false)
DEBUG_GET_ONCE_OPTION(pimax_slam_cams, "PIMAX_SLAM_CAMS", "0,1")
DEBUG_GET_ONCE_OPTION(pimax_slam_record, "PIMAX_SLAM_RECORD", NULL)
DEBUG_GET_ONCE_NUM_OPTION(pimax_slam_cam_offset_us, "PIMAX_SLAM_CAM_OFFSET_US", 0)
DEBUG_GET_ONCE_OPTION(pimax_slam_imu, "PIMAX_SLAM_IMU", NULL)
DEBUG_GET_ONCE_OPTION(pimax_cam_ae, "PIMAX_CAM_AE", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_slam_exposure_mid, "PIMAX_SLAM_EXPOSURE_MID", true)

enum ae_mode
{
	AE_OFF = 0,
	AE_DRY = 1,  //!< controller runs, nothing is sent
	AE_SEND = 2, //!< exposure push sent through pc->send
};

//! Basalt's world is z-up (gravity -z); OpenXR is y-up: rotate -90 deg about X (same fix as the Rift S driver).
static const struct xrt_quat SLAM_TO_YUP = {0.70710678f, 0.0f, 0.0f, -0.70710678f};

#define CAM_TRACE(pc, ...) U_LOG_IFL_T(pc->log_level, __VA_ARGS__)
#define CAM_DEBUG(pc, ...) U_LOG_IFL_D(pc->log_level, __VA_ARGS__)
#define CAM_INFO(pc, ...) U_LOG_IFL_I(pc->log_level, __VA_ARGS__)
#define CAM_WARN(pc, ...) U_LOG_IFL_W(pc->log_level, __VA_ARGS__)
#define CAM_ERROR(pc, ...) U_LOG_IFL_E(pc->log_level, __VA_ARGS__)

static_assert(sizeof(struct pimax_camera_clock_ref) == sizeof(pimax_cam_imu_clock_ref), "clock ref layout");

struct pimax_camera
{
	enum u_logging_level log_level;
	struct xrt_device *hmd;
	pimax_camera_clock_fn clock;
	void *clock_user;

	pimax_cam *cam;
	pimax_cam_calib calib;
	char source_desc[256];

	struct xrt_frame_context xfctx;
	struct os_thread_helper thread;
	bool thread_started;

	//! Mercury.
	bool hands;
	uint32_t ht_cams[2];
	struct t_stereo_camera_calibration *stereo;
	struct xrt_device *ht_raw; //!< Mercury's device; owned by ht_dev after wrapping
	struct xrt_device *ht_dev; //!< handed to the system
	bool ht_taken;
	struct xrt_frame_sink *entry[2]; //!< per view: split(Mercury, SLAM placeholder)
	struct xrt_frame_sink slam_stub[2];
	uint64_t slam_stub_frames[2];
	bool stats_ht;

	//! Debug GUI: the four tiles of every frame.
	struct u_sink_debug debug[PIMAX_CAM_TILES];

	//! Stats (capture thread writes; u_var reads, races are harmless).
	uint64_t frames, tracking_frames, pairs_pushed, imu_timed, errors, timeouts;
	float fps, tracking_fps, mercury_fps;
	int64_t win_start_ns;
	uint64_t win_frames, win_tracking, win_ht_updates;
	int64_t last_ht_ts;
	//! Hand activity per stats window (PIMAX_HT_STATS=1): polls, active polls, last wrist in camera-0 space.
	uint64_t win_polls, win_active[2];
	struct xrt_vec3 last_wrist[2];
	int64_t last_frame_ts;

	//! 6DoF (PIMAX_SLAM) and EuRoC recording (PIMAX_SLAM_RECORD).
	pimax_camera_orient_fn orient;
	void *orient_user;
	bool slam_on;          //!< tracker running
	bool record_on;        //!< recorder running
	uint32_t slam_cams[4]; //!< factory camera ids in tracker order
	uint32_t slam_n;
	int64_t slam_cam_offset_ns;
	bool exposure_mid; //!< tracker frame time = stamp + exposure/2 (the stamp marks the exposure start)
	struct t_slam_calibration slam_calib;
	struct xrt_tracked_slam *slam;
	struct xrt_slam_sinks *slam_sinks;
	struct xrt_slam_sinks *rec_sinks;
	char rec_path[512];
	struct os_mutex slam_lock; //!< supervisor + tracker queries (compositor and camera threads)
	bool slam_lock_init;
	pimax_cam_track *track;
	pimax_cam_track_out tout;
	double head_in_imu[3];
	int64_t last_poll_ns;
	uint64_t slam_frames, slam_untimed, imu_pushed;
	char slam_status[128];
	struct u_var_button recenter_btn;
	int64_t last_info_ns;

	//! Auto-exposure (PIMAX_CAM_AE): Rust controller, the HMD sends.
	pimax_camera_send_fn send;
	void *send_user;
	enum ae_mode ae_mode;
	pimax_cam_ae *ae;
	pimax_cam_ae_out ae_out;
	uint64_t ae_sent, ae_send_errors;
	char ae_status[160];
};

static void
slam_stub_push0(struct xrt_frame_sink *xfs, struct xrt_frame *xf)
{
	(void)xf;
	struct pimax_camera *pc = container_of(xfs, struct pimax_camera, slam_stub[0]);
	pc->slam_stub_frames[0]++;
}

static void
slam_stub_push1(struct xrt_frame_sink *xfs, struct xrt_frame *xf)
{
	(void)xf;
	struct pimax_camera *pc = container_of(xfs, struct pimax_camera, slam_stub[1]);
	pc->slam_stub_frames[1]++;
}

static bool
parse_u32_pair(const char *s, uint32_t out[2])
{
	unsigned a = 0, b = 0;
	if (s == NULL || sscanf(s, "%u,%u", &a, &b) != 2) {
		return false;
	}
	out[0] = a;
	out[1] = b;
	return true;
}

static void
mat9_to_quat(const double r[9], struct xrt_quat *out)
{
	struct xrt_matrix_3x3 m;
	for (int i = 0; i < 9; i++) {
		m.v[i] = (float)r[i];
	}
	math_quat_from_matrix_3x3(&m, out);
	math_quat_normalize(out);
}

/*!
 * Mercury view orientation from the calibration: the image "down" axis of the camera in the body frame (y up).
 * Image down pointing up = camera mounted upside down = CAMERA_ORIENTATION_180.
 */
static enum t_camera_orientation
orientation_of(struct pimax_camera *pc, uint32_t cam)
{
	double r[9], t[3];
	if (pimax_cam_calib_body_from_cam(&pc->calib, cam, r, t) != 0) {
		return CAMERA_ORIENTATION_0;
	}
	double down_y = r[4];  // column 1 (image +y), body y component
	double right_y = r[3]; // column 0 (image +x), body y component
	if (down_y > 0.5) {
		return CAMERA_ORIENTATION_180;
	}
	if (down_y < -0.5) {
		return CAMERA_ORIENTATION_0;
	}
	CAM_WARN(pc, "camera %u is rolled ~90 deg (image right y=%.2f); orientation 90/270 not verified, using 0", cam,
	         right_y);
	return CAMERA_ORIENTATION_0;
}

static struct t_stereo_camera_calibration *
make_stereo(struct pimax_camera *pc, uint32_t a, uint32_t b)
{
	struct t_stereo_camera_calibration *sc = NULL;
	t_stereo_camera_calibration_alloc(&sc, T_DISTORTION_FISHEYE_KB4);
	const uint32_t ids[2] = {a, b};
	for (int v = 0; v < 2; v++) {
		const pimax_cam_kb4 *k = &pc->calib.cameras[ids[v]].intr;
		struct t_camera_calibration *tc = &sc->view[v];
		tc->image_size_pixels.w = k->width;
		tc->image_size_pixels.h = k->height;
		t_camera_calibration_fill_intrinsics(tc, k->fx, k->fy, k->cx, k->cy);
		tc->distortion_model = T_DISTORTION_FISHEYE_KB4;
		tc->kb4.k1 = k->k[0];
		tc->kb4.k2 = k->k[1];
		tc->kb4.k3 = k->k[2];
		tc->kb4.k4 = k->k[3];
	}
	double r[9], t[3];
	if (pimax_cam_calib_stereo(&pc->calib, a, b, r, t) != 0) {
		CAM_ERROR(pc, "stereo extrinsic %u-%u: %s", a, b, pimax_cam_last_error());
		t_stereo_camera_calibration_reference(&sc, NULL);
		return NULL;
	}
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			sc->camera_rotation[i][j] = r[i * 3 + j];
		}
		sc->camera_translation[i] = t[i];
	}
	CAM_INFO(pc, "Mercury stereo: cameras %u (%s) + %u (%s), baseline %.1f mm", a, pc->calib.cameras[a].name, b,
	         pc->calib.cameras[b].name, sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]) * 1000.0);
	return sc;
}

/*!
 * offset for multi_create_tracking_override(ATTACHED): pose of the head in the space of Mercury's output frame
 * (view-0 camera with -Z forward, +Y up). hand_in_head = head_from_camCV * Rx(180) * hand_in_camXR.
 */
static bool
make_attach_offset(struct pimax_camera *pc, struct xrt_pose *out_offset)
{
	double r0[9], t0[3], r1[9], t1[3];
	if (pimax_cam_calib_body_from_cam(&pc->calib, pc->ht_cams[0], r0, t0) != 0 ||
	    pimax_cam_calib_body_from_cam(&pc->calib, pc->ht_cams[1], r1, t1) != 0) {
		CAM_WARN(pc, "calibration has no camera-IMU extrinsic; hands attached with identity");
		*out_offset = (struct xrt_pose)XRT_POSE_IDENTITY;
		return false;
	}
	// Head origin in the body (IMU) frame: between the two cameras, 5 cm behind them (estimate; see header).
	struct xrt_vec3 head = {(float)((t0[0] + t1[0]) / 2), (float)((t0[1] + t1[1]) / 2),
	                        (float)((t0[2] + t1[2]) / 2 + 0.05)};
	const char *ho = debug_get_option_pimax_ht_head_offset();
	if (ho != NULL) {
		float x, y, z;
		if (sscanf(ho, "%f,%f,%f", &x, &y, &z) == 3) {
			head = (struct xrt_vec3){x, y, z};
		} else {
			CAM_WARN(pc, "PIMAX_HT_HEAD_OFFSET=%s ignored (want x,y,z in metres)", ho);
		}
	}
	struct xrt_pose head_from_camcv;
	mat9_to_quat(r0, &head_from_camcv.orientation);
	head_from_camcv.position =
	    (struct xrt_vec3){(float)t0[0] - head.x, (float)t0[1] - head.y, (float)t0[2] - head.z};
	const struct xrt_pose rx180 = {.orientation = {.x = 1.0f, .y = 0.0f, .z = 0.0f, .w = 0.0f},
	                               .position = {0, 0, 0}};
	struct xrt_pose head_from_camxr;
	math_pose_transform(&head_from_camcv, &rx180, &head_from_camxr);
	math_pose_invert(&head_from_camxr, out_offset);
	CAM_INFO(pc,
	         "hands attached: head origin (%.3f %.3f %.3f) m in the IMU frame, camera %u at (%.3f %.3f %.3f) "
	         "from the head",
	         head.x, head.y, head.z, pc->ht_cams[0], head_from_camcv.position.x, head_from_camcv.position.y,
	         head_from_camcv.position.z);
	return true;
}

static uint64_t
frame_time(struct pimax_camera *pc, const pimax_cam_frame_info *fi, bool *out_imu_timed)
{
	*out_imu_timed = false;
	// A replay's device timestamps come from another session: use its rebased monotonic time instead.
	if ((fi->flags & PIMAX_CAM_FLAG_REPLAY) == 0 && pc->clock != NULL) {
		struct pimax_camera_clock_ref ref;
		if (pc->clock(pc->clock_user, &ref) && ref.valid) {
			pimax_cam_imu_clock_ref r;
			memcpy(&r, &ref, sizeof(r));
			uint64_t t = pimax_cam_dev_to_mono_ns(fi->dev_ts_us, &r);
			if (t != 0) {
				pc->imu_timed++;
				*out_imu_timed = true;
				return t;
			}
		}
	}
	return fi->mono_ts_ns;
}

static struct xrt_frame *
tile_frame(struct pimax_camera *pc, uint32_t tile, uint64_t ts, const pimax_cam_frame_info *fi)
{
	struct xrt_frame *xf = NULL;
	u_frame_create_one_off(XRT_FORMAT_L8, PIMAX_CAM_TILE_W, PIMAX_CAM_TILE_H, &xf);
	if (xf == NULL) {
		return NULL;
	}
	if (pimax_cam_copy_tile(pc->cam, tile, xf->data, (uint32_t)xf->stride) != 0) {
		CAM_ERROR(pc, "copy tile %u: %s", tile, pimax_cam_last_error());
		xrt_frame_reference(&xf, NULL);
		return NULL;
	}
	xf->timestamp = (int64_t)ts;
	xf->source_timestamp = (int64_t)(fi->dev_ts_us * 1000);
	xf->source_sequence = fi->counter;
	xf->source_id = tile;
	return xf;
}

static void
poll_mercury(struct pimax_camera *pc)
{
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	if (!pc->stats_ht || pc->ht_raw == NULL) {
		return;
	}
	// With HTA_PREDICTION_DISABLE=1 the async shim returns the timestamp of the last processed pair.
	// The raw Mercury device answers in the view-0 camera frame (-Z forward, +Y up), metres.
	const enum xrt_input_name names[2] = {XRT_INPUT_HT_UNOBSTRUCTED_LEFT, XRT_INPUT_HT_UNOBSTRUCTED_RIGHT};
	int64_t now = os_monotonic_get_ns();
	pc->win_polls++;
	for (int h = 0; h < 2; h++) {
		struct xrt_hand_joint_set set;
		int64_t ts = 0;
		if (xrt_device_get_hand_tracking(pc->ht_raw, names[h], now, &set, &ts) != XRT_SUCCESS) {
			continue;
		}
		if (h == 0 && ts != pc->last_ht_ts) {
			pc->last_ht_ts = ts;
			pc->win_ht_updates++;
		}
		if (set.is_active) {
			pc->win_active[h]++;
			pc->last_wrist[h] =
			    set.values.hand_joint_set_default[XRT_HAND_JOINT_WRIST].relation.pose.position;
		}
	}
#else
	(void)pc;
#endif
}

//! One tracking frame through the auto-exposure controller; sends only in AE_SEND mode.
static void
ae_step(struct pimax_camera *pc, int64_t now)
{
	if (pc->ae == NULL) {
		return;
	}
	pimax_cam_ae_out *o = &pc->ae_out;
	int r = pimax_cam_ae_update(pc->ae, pc->cam, (uint64_t)now, o);
	if (r < 0) {
		return;
	}
	if (o->changed) {
		CAM_DEBUG(pc, "AE: target exposure %u/%u %u/%u gain %u/%u %u/%u (frame %u/%u, mean %.0f/%.0f, sat %.3f/%.3f)",
		          o->exposure[0], o->exposure[2], o->exposure[1], o->exposure[3], o->gain[0], o->gain[2], o->gain[1],
		          o->gain[3], o->frame_exposure[0], o->frame_gain[0], o->mean[0], o->mean[1], o->sat[0], o->sat[1]);
	}
	if (r == 1 && pc->ae_mode == AE_SEND && pc->send != NULL) {
		if (pc->send(pc->send_user, PIMAX_CAM_AE_CMD, PIMAX_CAM_AE_OPT, o->payload, PIMAX_CAM_AE_PAYLOAD_LEN) == 0) {
			pc->ae_sent++;
		} else if (pc->ae_send_errors++ % 50 == 0) {
			CAM_WARN(pc, "AE: exposure push failed (%" PRIu64 " errors)", pc->ae_send_errors);
		}
	}
	snprintf(pc->ae_status, sizeof(pc->ae_status),
	         "%s: lower %u/%u mean %.0f, upper %u/%u mean %.0f; frame %u/%u; %" PRIu64 " changes, %" PRIu64 " sent",
	         pc->ae_mode == AE_SEND ? "on" : "dry", o->exposure[0], o->gain[0], o->mean[0], o->exposure[2],
	         o->gain[2], o->mean[1], o->frame_exposure[0], o->frame_gain[0], o->changes, pc->ae_sent);
}

static void
update_stats(struct pimax_camera *pc, int64_t now)
{
	if (pc->win_start_ns == 0) {
		pc->win_start_ns = now;
		return;
	}
	double dt = (double)(now - pc->win_start_ns) / 1e9;
	if (dt < 5.0) {
		return;
	}
	pc->fps = (float)(pc->win_frames / dt);
	pc->tracking_fps = (float)(pc->win_tracking / dt);
	pc->mercury_fps = (float)(pc->win_ht_updates / dt);
	pimax_cam_stats st = {0};
	pimax_cam_get_stats(pc->cam, &st);
	CAM_INFO(pc,
	         "camera: %.2f fps (tracking %.2f), total %" PRIu64 " frames, lost %" PRIu64 ", corrupt %" PRIu64
	         ", IMU-timed %" PRIu64 ", Mercury pairs pushed %" PRIu64 "%s",
	         pc->fps, pc->tracking_fps, st.frames, st.dropped, st.corrupt, pc->imu_timed, pc->pairs_pushed, "");
	if (pc->slam_on || pc->record_on) {
		CAM_INFO(pc, "slam: %" PRIu64 " frame sets, %" PRIu64 " IMU samples, %" PRIu64 " untimed frames skipped%s%s",
		         pc->slam_frames, pc->imu_pushed, pc->slam_untimed, pc->slam_on ? "; " : "",
		         pc->slam_on ? pc->slam_status : "");
	}
	if (pc->ae != NULL) {
		CAM_INFO(pc, "AE %s", pc->ae_status);
	}
	if (pc->slam_on && pc->track != NULL) {
		pimax_cam_track_info ti;
		os_mutex_lock(&pc->slam_lock);
		int ok = pimax_cam_track_get_info(pc->track, &ti);
		os_mutex_unlock(&pc->slam_lock);
		if (ok == 0) {
			CAM_INFO(pc,
			         "6DoF output: gyro bias (%.3f %.3f %.3f) deg/s, correction %.1f mm / %.2f deg (max %.1f mm / "
			         "%.2f deg), %" PRIu64 " blended, %" PRIu64 " snapped",
			         ti.gyro_bias[0] * 57.2958, ti.gyro_bias[1] * 57.2958, ti.gyro_bias[2] * 57.2958,
			         sqrt(ti.corr_pos[0] * ti.corr_pos[0] + ti.corr_pos[1] * ti.corr_pos[1] +
			              ti.corr_pos[2] * ti.corr_pos[2]) * 1000.0,
			         sqrt(ti.corr_rot[0] * ti.corr_rot[0] + ti.corr_rot[1] * ti.corr_rot[1] +
			              ti.corr_rot[2] * ti.corr_rot[2]) * 57.2958,
			         ti.max_corr_pos * 1000.0, ti.max_corr_rot * 57.2958, ti.blended, ti.snapped);
		}
	}
	if (pc->stats_ht) {
		CAM_INFO(pc, "mercury: %.2f processed pairs/s (SLAM placeholder got %" PRIu64 "/%" PRIu64 " frames)",
		         pc->mercury_fps, pc->slam_stub_frames[0], pc->slam_stub_frames[1]);
		for (int h = 0; h < 2; h++) {
			struct xrt_vec3 w = pc->last_wrist[h];
			CAM_INFO(pc,
			         "mercury: %s hand active in %" PRIu64 "/%" PRIu64
			         " polls%s (last wrist %.2f %.2f %.2f m, camera %u frame)",
			         h == 0 ? "left" : "right", pc->win_active[h], pc->win_polls,
			         pc->win_active[h] > 0 ? "" : ", none seen", w.x, w.y, w.z, pc->ht_cams[0]);
		}
	}
	pc->win_polls = pc->win_active[0] = pc->win_active[1] = 0;
	pc->win_start_ns = now;
	pc->win_frames = pc->win_tracking = pc->win_ht_updates = 0;
}

/*
 *
 * 6DoF: SLAM tracker, supervisor, recorder.
 *
 */

//! Tracker relation (z-up world) -> y-up world, in place.
static void
slam_rel_to_yup(struct xrt_space_relation *r)
{
	math_quat_rotate(&SLAM_TO_YUP, &r->pose.orientation, &r->pose.orientation);
	math_quat_rotate_vec3(&SLAM_TO_YUP, &r->pose.position, &r->pose.position);
	math_quat_rotate_vec3(&SLAM_TO_YUP, &r->linear_velocity, &r->linear_velocity);
	math_quat_rotate_vec3(&SLAM_TO_YUP, &r->angular_velocity, &r->angular_velocity);
}

static const char *
track_state_name(uint32_t s)
{
	switch (s) {
	case PIMAX_CAM_TRACK_TRACKING: return "6DoF tracking";
	case PIMAX_CAM_TRACK_LOST: return "lost (3DoF, position held)";
	default: return "waiting for the tracker (3DoF)";
	}
}

//! Feed the newest tracker pose to the supervisor. Caller holds slam_lock.
static void
slam_poll_locked(struct pimax_camera *pc, int64_t now_ns)
{
	pimax_cam_track_vio smp;
	const pimax_cam_track_vio *sp = NULL;
	int64_t ts = 0;
	struct xrt_space_relation r = XRT_SPACE_RELATION_ZERO;
	struct xrt_quat q3;
	if (t_slam_get_latest_pose(pc->slam, &ts, &r) && pc->orient != NULL && pc->orient(pc->orient_user, ts, &q3)) {
		slam_rel_to_yup(&r);
		smp.ts_ns = (uint64_t)ts;
		smp.pos[0] = r.pose.position.x;
		smp.pos[1] = r.pose.position.y;
		smp.pos[2] = r.pose.position.z;
		smp.q[0] = r.pose.orientation.x;
		smp.q[1] = r.pose.orientation.y;
		smp.q[2] = r.pose.orientation.z;
		smp.q[3] = r.pose.orientation.w;
		smp.vel[0] = r.linear_velocity.x;
		smp.vel[1] = r.linear_velocity.y;
		smp.vel[2] = r.linear_velocity.z;
		smp.q3dof[0] = q3.x;
		smp.q3dof[1] = q3.y;
		smp.q3dof[2] = q3.z;
		smp.q3dof[3] = q3.w;
		sp = &smp;
	}
	pimax_cam_track_out o;
	if (pimax_cam_track_update_vio(pc->track, (uint64_t)now_ns, sp, &o) != 0) {
		return;
	}
	pc->tout = o;
	pc->last_poll_ns = now_ns;
	if (o.events & PIMAX_CAM_TRACK_EV_ACQUIRED) {
		CAM_INFO(pc, "6DoF: %s (acquisition %" PRIu64 ")", o.reason, o.acquisitions);
	}
	if (o.events & PIMAX_CAM_TRACK_EV_LOST) {
		CAM_WARN(pc, "6DoF: lost (%s); 3DoF at the last position", o.reason);
	}
	if (o.events & PIMAX_CAM_TRACK_EV_RECENTERED) {
		CAM_INFO(pc, "6DoF: recentered");
	}
	if (o.events & PIMAX_CAM_TRACK_EV_RESET_WANTED) {
		CAM_WARN(pc, "6DoF: still lost, resetting the tracker (reset %" PRIu64 ")", o.resets);
		t_slam_reset(pc->slam);
	}
	snprintf(pc->slam_status, sizeof(pc->slam_status), "%s; %" PRIu64 " acquired, %" PRIu64 " lost, %s",
	         track_state_name(o.state), o.acquisitions, o.losses, o.reason);
}

bool
pimax_camera_wants_imu(const struct pimax_camera *pc)
{
	return pc != NULL && (pc->slam_on || pc->record_on);
}

void
pimax_camera_push_imu(struct pimax_camera *pc, int64_t ts_ns, const struct xrt_vec3 *accel_m_s2,
                      const struct xrt_vec3 *gyro_rad_s)
{
	if (!pimax_camera_wants_imu(pc)) {
		return;
	}
	struct xrt_imu_sample s = {
	    .timestamp_ns = ts_ns,
	    .accel_m_s2 = {accel_m_s2->x, accel_m_s2->y, accel_m_s2->z},
	    .gyro_rad_secs = {gyro_rad_s->x, gyro_rad_s->y, gyro_rad_s->z},
	};
	if (pc->slam_on) {
		xrt_sink_push_imu(pc->slam_sinks->imus[0], &s);
		// Same sample into the Rust predictor (rotation at IMU rate between tracker poses).
		os_mutex_lock(&pc->slam_lock);
		const double a3[3] = {accel_m_s2->x, accel_m_s2->y, accel_m_s2->z};
		const double g3[3] = {gyro_rad_s->x, gyro_rad_s->y, gyro_rad_s->z};
		pimax_cam_track_push_imu(pc->track, ts_ns, a3, g3);
		os_mutex_unlock(&pc->slam_lock);
	}
	if (pc->record_on) {
		xrt_sink_push_imu(pc->rec_sinks->imus[0], &s);
	}
	pc->imu_pushed++;
}

static void
rel_to_c(const struct xrt_space_relation *r, pimax_cam_rel *c)
{
	memset(c, 0, sizeof(*c));
	c->pos[0] = r->pose.position.x;
	c->pos[1] = r->pose.position.y;
	c->pos[2] = r->pose.position.z;
	c->q[0] = r->pose.orientation.x;
	c->q[1] = r->pose.orientation.y;
	c->q[2] = r->pose.orientation.z;
	c->q[3] = r->pose.orientation.w;
	c->lin_vel[0] = r->linear_velocity.x;
	c->lin_vel[1] = r->linear_velocity.y;
	c->lin_vel[2] = r->linear_velocity.z;
	c->ang_vel[0] = r->angular_velocity.x;
	c->ang_vel[1] = r->angular_velocity.y;
	c->ang_vel[2] = r->angular_velocity.z;
	c->flags = (uint32_t)r->relation_flags;
}

static void
rel_from_c(const pimax_cam_rel *c, struct xrt_space_relation *r)
{
	r->pose.position = (struct xrt_vec3){(float)c->pos[0], (float)c->pos[1], (float)c->pos[2]};
	r->pose.orientation = (struct xrt_quat){(float)c->q[0], (float)c->q[1], (float)c->q[2], (float)c->q[3]};
	r->linear_velocity = (struct xrt_vec3){(float)c->lin_vel[0], (float)c->lin_vel[1], (float)c->lin_vel[2]};
	r->angular_velocity = (struct xrt_vec3){(float)c->ang_vel[0], (float)c->ang_vel[1], (float)c->ang_vel[2]};
	r->relation_flags = (enum xrt_space_relation_flags)c->flags;
}

bool
pimax_camera_head_relation(struct pimax_camera *pc,
                           int64_t at_ns,
                           const struct xrt_space_relation *rel3dof,
                           struct xrt_space_relation *out)
{
	if (pc == NULL || !pc->slam_on) {
		return false;
	}
	// Rust (pimax_cam_track_head): IMU-rate prediction from the newest tracker pose with blended corrections while
	// tracking; 3DoF orientation (heading kept continuous) at the held position otherwise.
	pimax_cam_rel r3, ro;
	rel_to_c(rel3dof, &r3);
	os_mutex_lock(&pc->slam_lock);
	int64_t now = os_monotonic_get_ns();
	if (now - pc->last_poll_ns > U_TIME_1MS_IN_NS * 4) {
		slam_poll_locked(pc, now);
	}
	int st = pimax_cam_track_head(pc->track, (uint64_t)now, at_ns, &r3, &ro);
	os_mutex_unlock(&pc->slam_lock);
	if (st < 0) {
		return false;
	}
	*out = *rel3dof;
	rel_from_c(&ro, out);
	math_quat_normalize(&out->pose.orientation);
	return true;
}

static void
recenter_cb(void *ptr)
{
	struct pimax_camera *pc = (struct pimax_camera *)ptr;
	if (pc->track != NULL) {
		os_mutex_lock(&pc->slam_lock);
		pimax_cam_track_recenter(pc->track);
		os_mutex_unlock(&pc->slam_lock);
	}
}

//! Push one IMU-timed tracking frame (tiles in tracker order) to the tracker and/or the recorder. @p exp_us =
//! exposure of the first tracker camera (one timestamp per frame set: Basalt wants them identical).
static void
push_slam_frames(struct pimax_camera *pc, struct xrt_frame *tiles[PIMAX_CAM_TILES], int64_t ts, uint32_t exp_us)
{
	// Mid-exposure time: the frame stamp marks the exposure start (image rows start exposure/38.8 us later and
	// Pimax's factory delta 1.638 ms = half its typical 3.3 ms exposure; analysis/slam-exposure.md).
	int64_t adj = pc->slam_cam_offset_ns + (pc->exposure_mid ? (int64_t)exp_us * 500 : 0);
	for (uint32_t i = 0; i < pc->slam_n; i++) {
		if (tiles[pc->slam_cams[i]] == NULL) {
			return; // all or nothing: the tracker wants one frame per camera per timestamp
		}
	}
	for (uint32_t i = 0; i < pc->slam_n; i++) {
		// Shared frames: set the tracker timestamp on a private copy only when an offset is configured.
		struct xrt_frame *xf = tiles[pc->slam_cams[i]];
		struct xrt_frame *own = NULL;
		if (adj != 0) {
			u_frame_clone(xf, &own);
			own->timestamp = ts + adj;
			xf = own;
		}
		if (pc->slam_on) {
			xrt_sink_push_frame(pc->slam_sinks->cams[i], xf);
		}
		if (pc->record_on) {
			xrt_sink_push_frame(pc->rec_sinks->cams[i], xf);
		}
		xrt_frame_reference(&own, NULL);
	}
	pc->slam_frames++;
	if (pc->slam_on) {
		os_mutex_lock(&pc->slam_lock);
		slam_poll_locked(pc, os_monotonic_get_ns());
		os_mutex_unlock(&pc->slam_lock);
	}
}

static void
fill_slam_calibration(struct pimax_camera *pc, const pimax_cam_slam_imu *imu)
{
	struct t_slam_calibration *c = &pc->slam_calib;
	memset(c, 0, sizeof(*c));
	c->cam_count = (int)pc->slam_n;
	for (uint32_t i = 0; i < pc->slam_n; i++) {
		const pimax_cam_kb4 *k = &pc->calib.cameras[pc->slam_cams[i]].intr;
		struct t_slam_camera_calibration *sc = &c->cams[i];
		struct t_camera_calibration *tc = &sc->base;
		tc->image_size_pixels.w = k->width;
		tc->image_size_pixels.h = k->height;
		t_camera_calibration_fill_intrinsics(tc, k->fx, k->fy, k->cx, k->cy);
		tc->distortion_model = T_DISTORTION_FISHEYE_KB4;
		tc->kb4.k1 = k->k[0];
		tc->kb4.k2 = k->k[1];
		tc->kb4.k3 = k->k[2];
		tc->kb4.k4 = k->k[3];
		double m[16];
		pimax_cam_slam_t_imu_cam(&pc->calib, pc->slam_cams[i], m); // checked in setup_slam
		for (int j = 0; j < 16; j++) {
			sc->T_imu_cam.v[j] = (float)m[j];
		}
		sc->frequency = 25.0;
	}
	struct t_imu_calibration *ic = &c->imu.base;
	for (int a = 0; a < 3; a++) {
		ic->accel.transform[a][a] = 1.0;
		ic->gyro.transform[a][a] = 1.0;
		ic->accel.noise_std[a] = imu->accel_noise_std[a];
		ic->accel.bias_std[a] = imu->accel_bias_std[a];
		ic->gyro.noise_std[a] = imu->gyro_noise_std[a];
		ic->gyro.bias_std[a] = imu->gyro_bias_std[a];
	}
	c->imu.frequency = imu->rate_hz;
}

static void
default_record_path(char *out, size_t len)
{
	const char *home = getenv("HOME");
	time_t now = time(NULL);
	struct tm tm;
	localtime_r(&now, &tm);
	char stamp[32];
	strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm);
	snprintf(out, len, "%s/.local/share/pimax/slam-recordings/%s", home != NULL ? home : ".", stamp);
}

//! PIMAX_SLAM / PIMAX_SLAM_RECORD. Returns false only on a configuration error.
static bool
setup_slam(struct pimax_camera *pc, bool want_slam, const char *record)
{
	int n = pimax_cam_slam_parse_cams(debug_get_option_pimax_slam_cams(), pc->calib.camera_count, pc->slam_cams,
	                                  4);
	if (n <= 0) {
		CAM_ERROR(pc, "PIMAX_SLAM_CAMS: %s", pimax_cam_last_error());
		return false;
	}
	pc->slam_n = (uint32_t)n;
	double m[16];
	for (uint32_t i = 0; i < pc->slam_n; i++) {
		if (pimax_cam_slam_t_imu_cam(&pc->calib, pc->slam_cams[i], m) != 0) {
			CAM_ERROR(pc, "SLAM camera %u: %s", pc->slam_cams[i], pimax_cam_last_error());
			return false;
		}
	}
	pc->slam_cam_offset_ns = (int64_t)debug_get_num_option_pimax_slam_cam_offset_us() * 1000;
	pc->exposure_mid = debug_get_bool_option_pimax_slam_exposure_mid();

	pimax_cam_slam_imu imu;
	int ir = pimax_cam_slam_imu_load(debug_get_option_pimax_slam_imu(), &imu);
	if (ir < 0) {
		CAM_WARN(pc, "IMU noise file: %s (using defaults)", pimax_cam_last_error());
	}
	fill_slam_calibration(pc, &imu);
	if (pimax_cam_slam_default_head(&pc->calib, pc->head_in_imu) != 0) {
		pc->head_in_imu[0] = pc->head_in_imu[1] = pc->head_in_imu[2] = 0.0;
	}
	const char *ho = debug_get_option_pimax_ht_head_offset();
	if (ho != NULL) {
		double x, y, z;
		if (sscanf(ho, "%lf,%lf,%lf", &x, &y, &z) == 3) {
			pc->head_in_imu[0] = x;
			pc->head_in_imu[1] = y;
			pc->head_in_imu[2] = z;
		}
	}
	char names[160] = {0};
	for (uint32_t i = 0; i < pc->slam_n; i++) {
		size_t l = strlen(names);
		snprintf(names + l, sizeof(names) - l, "%s%u (%s)", i ? ", " : "", pc->slam_cams[i],
		         pc->calib.cameras[pc->slam_cams[i]].name);
	}
	CAM_INFO(pc,
	         "SLAM sensors: cameras %s at 25 Hz, IMU 1 kHz, noise from %s, camera offset %" PRId64
	         " us, frame time at mid-exposure %s",
	         names, imu.source, pc->slam_cam_offset_ns / 1000, pc->exposure_mid ? "on" : "off");

	if (record != NULL && record[0] != '\0' && strcmp(record, "0") != 0) {
		if (strcmp(record, "1") == 0) {
			default_record_path(pc->rec_path, sizeof(pc->rec_path));
		} else {
			snprintf(pc->rec_path, sizeof(pc->rec_path), "%s", record);
		}
		char note[256];
		snprintf(note, sizeof(note),
		         "Monado pimax driver EuRoC recording; camera offset %" PRId64
		         " us applied; frame time = stamp + exposure/2: %s",
		         pc->slam_cam_offset_ns / 1000, pc->exposure_mid ? "yes" : "no");
		if (pimax_cam_slam_write_basalt(&pc->calib, pc->slam_cams, pc->slam_n, 25.0, 0, &imu, pc->rec_path,
		                                note) != 0) {
			CAM_ERROR(pc, "recording directory %s: %s", pc->rec_path, pimax_cam_last_error());
		} else {
			// The recorder appends _<datetime> to its prefix: <dir>/euroc_<datetime>/mav0/...
			char prefix[600];
			snprintf(prefix, sizeof(prefix), "%s/euroc", pc->rec_path);
			pc->rec_sinks = euroc_recorder_create(&pc->xfctx, prefix, (int)pc->slam_n, 1, true);
			pc->record_on = pc->rec_sinks != NULL;
			CAM_INFO(pc, "EuRoC recording %s: %s/euroc_<datetime> (calibration in %s/pimax-basalt-calib.json)",
			         pc->record_on ? "to" : "FAILED", pc->rec_path, pc->rec_path);
		}
	}

	if (!want_slam) {
		return true;
	}
#ifdef XRT_FEATURE_SLAM
	struct t_slam_tracker_config config = {0};
	t_slam_fill_default_config(&config);
	config.cam_count = (int)pc->slam_n;
	config.slam_calib = &pc->slam_calib;
	config.submit_from_start = true;
	struct xrt_slam_sinks *sinks = NULL;
	if (t_slam_create(&pc->xfctx, &config, &pc->slam, &sinks) != 0 || sinks == NULL) {
		CAM_ERROR(pc, "SLAM tracker could not be created (VIT library '%s' missing? set VIT_SYSTEM_LIBRARY_PATH); "
		          "staying 3DoF", config.vit_system_library_path);
		pc->slam = NULL;
		return true;
	}
	if (t_slam_start(pc->slam) != 0) {
		CAM_ERROR(pc, "SLAM tracker did not start; staying 3DoF");
		return true;
	}
	pc->slam_sinks = sinks;
	pc->track = pimax_cam_track_create(NULL, pc->head_in_imu);
	if (pc->track == NULL) {
		CAM_ERROR(pc, "6DoF supervisor: %s", pimax_cam_last_error());
		return true;
	}
	snprintf(pc->slam_status, sizeof(pc->slam_status), "%s", track_state_name(PIMAX_CAM_TRACK_INIT));
	pc->slam_on = true;
	CAM_INFO(pc, "6DoF: SLAM tracker up, head origin (%.3f %.3f %.3f) m in the IMU frame; 3DoF until it is trusted",
	         pc->head_in_imu[0], pc->head_in_imu[1], pc->head_in_imu[2]);
#else
	CAM_ERROR(pc, "PIMAX_SLAM=1 but Monado was built without XRT_FEATURE_SLAM; staying 3DoF");
#endif
	return true;
}

static void *
capture_thread(void *ptr)
{
	struct pimax_camera *pc = (struct pimax_camera *)ptr;
	U_TRACE_SET_THREAD_NAME("Pimax camera");
	os_thread_name_self("pimax-camera");

	uint32_t consecutive_errors = 0;
	while (os_thread_helper_is_running(&pc->thread)) {
		pimax_cam_frame_info fi;
		int r = pimax_cam_next(pc->cam, 200, &fi);
		int64_t now = os_monotonic_get_ns();
		poll_mercury(pc);
		update_stats(pc, now);
		if (r == 0) {
			pc->timeouts++;
			continue;
		}
		if (r < 0) {
			pc->errors++;
			if (consecutive_errors++ % 50 == 0) {
				CAM_ERROR(pc, "camera: %s", pimax_cam_last_error());
			}
			os_nanosleep(U_TIME_1MS_IN_NS * 100);
			continue;
		}
		consecutive_errors = 0;
		pc->frames++;
		pc->win_frames++;
		bool imu_timed = false;
		uint64_t ts = frame_time(pc, &fi, &imu_timed);
		pc->last_frame_ts = (int64_t)ts;

		// Debug GUI: every tile of every frame (tracking and LED exposure).
		for (uint32_t t = 0; t < PIMAX_CAM_TILES; t++) {
			if (u_sink_debug_is_active(&pc->debug[t])) {
				struct xrt_frame *xf = tile_frame(pc, t, ts, &fi);
				if (xf != NULL) {
					u_sink_debug_push_frame(&pc->debug[t], xf);
					xrt_frame_reference(&xf, NULL);
				}
			}
		}

		if (fi.kind != PIMAX_CAM_KIND_TRACKING) {
			continue;
		}
		pc->tracking_frames++;
		pc->win_tracking++;
		ae_step(pc, now);
		bool want_slam = pc->slam_on || pc->record_on;
		bool want_ht = pc->entry[0] != NULL && pc->entry[1] != NULL;
		if (!want_slam && !want_ht) {
			continue;
		}
		// One frame per needed tile, shared (read-only) by Mercury, the tracker and the recorder.
		struct xrt_frame *tiles[PIMAX_CAM_TILES] = {NULL};
		bool need[PIMAX_CAM_TILES] = {false};
		if (want_ht) {
			need[pc->ht_cams[0]] = need[pc->ht_cams[1]] = true;
		}
		if (want_slam && imu_timed) {
			for (uint32_t i = 0; i < pc->slam_n; i++) {
				need[pc->slam_cams[i]] = true;
			}
		} else if (want_slam) {
			pc->slam_untimed++; // the tracker needs the IMU timeline (no replays, no IMU yet)
		}
		for (uint32_t t = 0; t < PIMAX_CAM_TILES; t++) {
			if (need[t]) {
				tiles[t] = tile_frame(pc, t, ts, &fi);
			}
		}
		if (want_ht && tiles[pc->ht_cams[0]] != NULL && tiles[pc->ht_cams[1]] != NULL) {
			// Mercury's async shim requires left then right, from one thread.
			xrt_sink_push_frame(pc->entry[0], tiles[pc->ht_cams[0]]);
			xrt_sink_push_frame(pc->entry[1], tiles[pc->ht_cams[1]]);
			pc->pairs_pushed++;
		}
		if (want_slam && imu_timed) {
			push_slam_frames(pc, tiles, (int64_t)ts, fi.exposure[pc->slam_cams[0]]);
		}
		for (uint32_t t = 0; t < PIMAX_CAM_TILES; t++) {
			xrt_frame_reference(&tiles[t], NULL);
		}
	}
	return NULL;
}

static bool
setup_mercury(struct pimax_camera *pc)
{
#ifdef XRT_BUILD_DRIVER_HANDTRACKING
	if (!parse_u32_pair(debug_get_option_pimax_ht_cams(), pc->ht_cams) ||
	    pc->ht_cams[0] >= pc->calib.camera_count || pc->ht_cams[1] >= pc->calib.camera_count ||
	    pc->ht_cams[0] == pc->ht_cams[1]) {
		CAM_ERROR(pc, "PIMAX_HT_CAMS=%s: want two different camera ids < %u", debug_get_option_pimax_ht_cams(),
		          pc->calib.camera_count);
		return false;
	}
	pc->stereo = make_stereo(pc, pc->ht_cams[0], pc->ht_cams[1]);
	if (pc->stereo == NULL) {
		return false;
	}

	struct t_camera_extra_info extra = {0};
	uint32_t orient_env[2];
	for (int v = 0; v < 2; v++) {
		extra.views[v].boundary_type = HT_IMAGE_BOUNDARY_NONE;
		extra.views[v].camera_orientation = orientation_of(pc, pc->ht_cams[v]);
	}
	if (parse_u32_pair(debug_get_option_pimax_ht_orient(), orient_env)) {
		for (int v = 0; v < 2; v++) {
			extra.views[v].camera_orientation = (enum t_camera_orientation)orient_env[v];
		}
	}
	CAM_INFO(pc, "Mercury view orientation: %d, %d deg", (int)extra.views[0].camera_orientation,
	         (int)extra.views[1].camera_orientation);

	struct t_hand_tracking_create_info create_info = {.cams_info = extra, .masks_sink = NULL};
	struct xrt_slam_sinks *hand_sinks = NULL;
	struct xrt_device *ht = NULL;
	if (ht_device_create(&pc->xfctx, pc->stereo, create_info, &hand_sinks, &ht) != 0 || ht == NULL ||
	    hand_sinks == NULL) {
		CAM_ERROR(pc, "Mercury hand tracker could not be created (ONNX models in ~/.local/share/monado?)");
		return false;
	}
	pc->ht_raw = ht;

	// Entry per view: split to Mercury and to the (future) SLAM consumer.
	pc->slam_stub[0].push_frame = slam_stub_push0;
	pc->slam_stub[1].push_frame = slam_stub_push1;
	for (int v = 0; v < 2; v++) {
		u_sink_split_create(&pc->xfctx, hand_sinks->cams[v], &pc->slam_stub[v], &pc->entry[v]);
	}

	struct xrt_pose offset;
	make_attach_offset(pc, &offset);
	pc->ht_dev = multi_create_tracking_override(XRT_TRACKING_OVERRIDE_ATTACHED, ht, pc->hmd,
	                                            XRT_INPUT_GENERIC_HEAD_POSE, &offset);
	if (pc->ht_dev == NULL) {
		CAM_ERROR(pc, "could not attach the hand tracker to the HMD");
		xrt_device_destroy(&pc->ht_raw);
		return false;
	}
	snprintf(pc->ht_dev->str, XRT_DEVICE_NAME_LEN, "Pimax Dream Air Mercury hands");
	snprintf(pc->ht_dev->serial, XRT_DEVICE_NAME_LEN, "Pimax Dream Air Mercury hands");
	return true;
#else
	CAM_ERROR(pc, "PIMAX_HANDS=mercury but Monado was built without Mercury (needs ONNX Runtime + OpenCV)");
	(void)pc;
	return false;
#endif
}

struct pimax_camera *
pimax_camera_create(struct xrt_device *hmd,
                    enum u_logging_level log_level,
                    pimax_camera_clock_fn clock,
                    void *clock_user,
                    pimax_camera_orient_fn orient,
                    void *orient_user,
                    pimax_camera_send_fn send,
                    void *send_user)
{
	const char *hands = debug_get_option_pimax_hands();
	bool want_hands = hands != NULL && strcmp(hands, "mercury") == 0;
	if (hands != NULL && !want_hands && strcmp(hands, "0") != 0 && strcmp(hands, "off") != 0) {
		U_LOG_IFL_W(log_level, "PIMAX_HANDS=%s unknown (only \"mercury\")", hands);
	}
	bool want_slam = debug_get_bool_option_pimax_slam();
	const char *record = debug_get_option_pimax_slam_record();
	bool want_record = record != NULL && record[0] != '\0' && strcmp(record, "0") != 0;
	const char *ae = debug_get_option_pimax_cam_ae();
	enum ae_mode ae_mode = AE_OFF;
	if (ae != NULL && strcmp(ae, "dry") == 0) {
		ae_mode = AE_DRY;
	} else if (ae != NULL && strcmp(ae, "1") == 0) {
		ae_mode = AE_SEND;
	} else if (ae != NULL && ae[0] != '\0' && strcmp(ae, "0") != 0) {
		U_LOG_IFL_W(log_level, "PIMAX_CAM_AE=%s unknown (0, dry or 1); auto-exposure off", ae);
	}
	if (!want_hands && !want_slam && !want_record && ae_mode == AE_OFF && !debug_get_bool_option_pimax_cam()) {
		return NULL;
	}

	struct pimax_camera *pc = U_TYPED_CALLOC(struct pimax_camera);
	pc->log_level = log_level;
	pc->hmd = hmd;
	pc->clock = clock;
	pc->clock_user = clock_user;
	pc->orient = orient;
	pc->orient_user = orient_user;
	pc->send = send;
	pc->send_user = send_user;
	if (os_mutex_init(&pc->slam_lock) == 0) {
		pc->slam_lock_init = true;
	}
	pc->stats_ht = debug_get_bool_option_pimax_ht_stats();
	for (int t = 0; t < PIMAX_CAM_TILES; t++) {
		u_sink_debug_init(&pc->debug[t]);
	}
	os_thread_helper_init(&pc->thread);

	char used[512] = {0};
	if (pimax_cam_calib_load(debug_get_option_pimax_cam_calib(), &pc->calib, used, sizeof(used)) != 0) {
		CAM_ERROR(pc, "camera calibration: %s", pimax_cam_last_error());
		pimax_camera_destroy(&pc);
		return NULL;
	}
	CAM_INFO(pc, "camera calibration %s (%u cameras, %s)", used, pc->calib.camera_count, pc->calib.device_uid);

	const char *src = debug_get_option_pimax_cam_source();
	pimax_cam_open_params params = {0};
	if (src != NULL && src[0] != '\0' && strncmp(src, "/dev/", 5) != 0) {
		params.replay_flags = PIMAX_CAM_REPLAY_LOOP | PIMAX_CAM_REPLAY_REALTIME;
	}
	pc->cam = pimax_cam_open(src, &params);
	if (pc->cam == NULL) {
		CAM_ERROR(pc, "camera: %s", pimax_cam_last_error());
		pimax_camera_destroy(&pc);
		return NULL;
	}
	pimax_cam_describe(pc->cam, pc->source_desc, sizeof(pc->source_desc));
	CAM_INFO(pc, "camera source: %s (pimax-cam %s)", pc->source_desc, pimax_cam_version());

	const char *rec = debug_get_option_pimax_cam_record();
	if (rec != NULL && rec[0] != '\0') {
		if (pimax_cam_record_start(pc->cam, rec, 0) == 0) {
			CAM_INFO(pc, "recording raw camera frames to %s", rec);
		} else {
			CAM_WARN(pc, "recording: %s", pimax_cam_last_error());
		}
	}

	if (want_hands && !setup_mercury(pc)) {
		CAM_WARN(pc, "continuing without hand tracking");
	}
	pc->hands = pc->ht_dev != NULL;

	if ((want_slam || want_record) && pc->slam_lock_init && !setup_slam(pc, want_slam, record)) {
		CAM_WARN(pc, "continuing without 6DoF/recording");
	}

	if (ae_mode != AE_OFF) {
		pc->ae = pimax_cam_ae_create();
		if (pc->ae == NULL) {
			CAM_ERROR(pc, "auto-exposure: %s", pimax_cam_last_error());
		} else if (ae_mode == AE_SEND && send == NULL) {
			CAM_WARN(pc, "PIMAX_CAM_AE=1 but no headset command channel (no device?): dry run only");
			pc->ae_mode = AE_DRY;
		} else {
			pc->ae_mode = ae_mode;
		}
		if (pc->ae != NULL) {
			snprintf(pc->ae_status, sizeof(pc->ae_status), "%s, waiting for tracking frames",
			         pc->ae_mode == AE_SEND ? "on" : "dry");
			CAM_WARN(pc, "auto-exposure %s: %s", pc->ae_mode == AE_SEND ? "ON" : "DRY RUN",
			         pc->ae_mode == AE_SEND ? "sending the exposure push (HID 0x51) to the headset"
			                                : "computing only, nothing is sent (PIMAX_CAM_AE=1 sends)");
		}
	}

	u_var_add_root(pc, "Pimax cameras", true);
	u_var_add_log_level(pc, &pc->log_level, "log_level");
	u_var_add_ro_text(pc, pc->source_desc, "source");
	u_var_add_ro_f32(pc, &pc->fps, "fps");
	u_var_add_ro_f32(pc, &pc->tracking_fps, "tracking fps");
	u_var_add_ro_u64(pc, &pc->frames, "frames");
	u_var_add_ro_u64(pc, &pc->pairs_pushed, "Mercury pairs pushed");
	if (pc->slam_on || pc->record_on) {
		u_var_add_gui_header(pc, NULL, "6DoF / SLAM");
		u_var_add_ro_text(pc, pc->slam_status, "6DoF state");
		u_var_add_ro_text(pc, pc->rec_path, "EuRoC recording");
		u_var_add_ro_u64(pc, &pc->slam_frames, "SLAM frame sets");
		u_var_add_ro_u64(pc, &pc->imu_pushed, "SLAM IMU samples");
		pc->recenter_btn.cb = recenter_cb;
		pc->recenter_btn.ptr = pc;
		u_var_add_button(pc, &pc->recenter_btn, "Recenter 6DoF (head to origin)");
	}
	if (pc->ae != NULL) {
		u_var_add_gui_header(pc, NULL, "Auto-exposure");
		u_var_add_ro_text(pc, pc->ae_status, "AE");
	}
	for (int t = 0; t < PIMAX_CAM_TILES; t++) {
		char name[48];
		snprintf(name, sizeof(name), "tile %d (%s)", t, pc->calib.cameras[t].name);
		u_var_add_sink_debug(pc, &pc->debug[t], name);
	}

	if (os_thread_helper_start(&pc->thread, capture_thread, pc) != 0) {
		CAM_ERROR(pc, "could not start the capture thread");
		pimax_camera_destroy(&pc);
		return NULL;
	}
	pc->thread_started = true;
	CAM_INFO(pc, "camera pipeline up%s%s%s", pc->hands ? " with Mercury hand tracking" : "",
	         pc->slam_on ? " with 6DoF SLAM" : "", pc->record_on ? " with EuRoC recording" : "");
	return pc;
}

struct xrt_device *
pimax_camera_take_hand_device(struct pimax_camera *pc)
{
	if (pc == NULL || pc->ht_dev == NULL || pc->ht_taken) {
		return NULL;
	}
	pc->ht_taken = true;
	return pc->ht_dev;
}

void
pimax_camera_destroy(struct pimax_camera **pc_ptr)
{
	struct pimax_camera *pc = *pc_ptr;
	if (pc == NULL) {
		return;
	}
	*pc_ptr = NULL;

	if (pc->thread_started) {
		os_thread_helper_stop_and_wait(&pc->thread);
	}
	os_thread_helper_destroy(&pc->thread);

	if (pc->cam != NULL) {
		int64_t n = pimax_cam_record_stop(pc->cam);
		if (n > 0) {
			CAM_INFO(pc, "recorded %" PRId64 " frames", n);
		}
	}

	// Mercury's async shim and the splits live in our frame context.
	xrt_frame_context_destroy_nodes(&pc->xfctx);

	if (pc->ht_dev != NULL && !pc->ht_taken) {
		// Never handed to the system: we own it (the wrapper destroys Mercury's device too).
		xrt_device_destroy(&pc->ht_dev);
	}
	t_stereo_camera_calibration_reference(&pc->stereo, NULL);
	if (pc->ae != NULL) {
		CAM_INFO(pc, "AE %s", pc->ae_status);
		pimax_cam_ae_destroy(pc->ae);
		pc->ae = NULL;
	}
	pimax_cam_track_destroy(pc->track);
	pc->track = NULL;
	if (pc->slam_lock_init) {
		os_mutex_destroy(&pc->slam_lock);
	}
	if (pc->record_on) {
		CAM_INFO(pc, "EuRoC recording finished: %s (%" PRIu64 " frame sets, %" PRIu64 " IMU samples)",
		         pc->rec_path, pc->slam_frames, pc->imu_pushed);
	}

	if (pc->cam != NULL) {
		pimax_cam_close(pc->cam);
		pc->cam = NULL;
	}
	u_var_remove_root(pc);
	for (int t = 0; t < PIMAX_CAM_TILES; t++) {
		u_sink_debug_destroy(&pc->debug[t]);
	}
	free(pc);
}

#endif // PIMAX_HAVE_CAM
