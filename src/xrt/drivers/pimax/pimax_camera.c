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
 *                                     -> SLAM placeholder sink (counts frames; Basalt goes here later)
 *   Mercury device -> multi tracking override attached to the HMD head pose -> XR_EXT_hand_tracking.
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

struct pimax_camera *
pimax_camera_create(struct xrt_device *hmd,
                    enum u_logging_level log_level,
                    pimax_camera_clock_fn clock,
                    void *clock_user)
{
	(void)hmd;
	(void)clock;
	(void)clock_user;
	const char *hands = debug_get_option_pimax_hands_stub();
	if (hands != NULL || debug_get_bool_option_pimax_cam_stub()) {
		U_LOG_IFL_W(log_level, "PIMAX_HANDS/PIMAX_CAM set, but Monado was built without -DPIMAX_CAM=ON");
	}
	return NULL;
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

DEBUG_GET_ONCE_OPTION(pimax_hands, "PIMAX_HANDS", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_cam, "PIMAX_CAM", false)
DEBUG_GET_ONCE_OPTION(pimax_cam_source, "PIMAX_CAM_SOURCE", NULL)
DEBUG_GET_ONCE_OPTION(pimax_cam_record, "PIMAX_CAM_RECORD", NULL)
DEBUG_GET_ONCE_OPTION(pimax_cam_calib, "PIMAX_CAM_CALIB", NULL)
DEBUG_GET_ONCE_OPTION(pimax_ht_cams, "PIMAX_HT_CAMS", "0,1")
DEBUG_GET_ONCE_OPTION(pimax_ht_orient, "PIMAX_HT_ORIENT", NULL)
DEBUG_GET_ONCE_OPTION(pimax_ht_head_offset, "PIMAX_HT_HEAD_OFFSET", NULL)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ht_stats, "PIMAX_HT_STATS", false)

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
frame_time(struct pimax_camera *pc, const pimax_cam_frame_info *fi)
{
	if (pc->clock != NULL) {
		struct pimax_camera_clock_ref ref;
		if (pc->clock(pc->clock_user, &ref) && ref.valid) {
			pimax_cam_imu_clock_ref r;
			memcpy(&r, &ref, sizeof(r));
			uint64_t t = pimax_cam_dev_to_mono_ns(fi->dev_ts_us, &r);
			if (t != 0) {
				pc->imu_timed++;
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
		uint64_t ts = frame_time(pc, &fi);
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
		if (pc->entry[0] == NULL || pc->entry[1] == NULL) {
			continue;
		}
		struct xrt_frame *views[2] = {NULL, NULL};
		views[0] = tile_frame(pc, pc->ht_cams[0], ts, &fi);
		views[1] = tile_frame(pc, pc->ht_cams[1], ts, &fi);
		if (views[0] != NULL && views[1] != NULL) {
			// Mercury's async shim requires left then right, from one thread.
			xrt_sink_push_frame(pc->entry[0], views[0]);
			xrt_sink_push_frame(pc->entry[1], views[1]);
			pc->pairs_pushed++;
		}
		xrt_frame_reference(&views[0], NULL);
		xrt_frame_reference(&views[1], NULL);
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
                    void *clock_user)
{
	const char *hands = debug_get_option_pimax_hands();
	bool want_hands = hands != NULL && strcmp(hands, "mercury") == 0;
	if (hands != NULL && !want_hands && strcmp(hands, "0") != 0 && strcmp(hands, "off") != 0) {
		U_LOG_IFL_W(log_level, "PIMAX_HANDS=%s unknown (only \"mercury\")", hands);
	}
	if (!want_hands && !debug_get_bool_option_pimax_cam()) {
		return NULL;
	}

	struct pimax_camera *pc = U_TYPED_CALLOC(struct pimax_camera);
	pc->log_level = log_level;
	pc->hmd = hmd;
	pc->clock = clock;
	pc->clock_user = clock_user;
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

	u_var_add_root(pc, "Pimax cameras", true);
	u_var_add_log_level(pc, &pc->log_level, "log_level");
	u_var_add_ro_text(pc, pc->source_desc, "source");
	u_var_add_ro_f32(pc, &pc->fps, "fps");
	u_var_add_ro_f32(pc, &pc->tracking_fps, "tracking fps");
	u_var_add_ro_u64(pc, &pc->frames, "frames");
	u_var_add_ro_u64(pc, &pc->pairs_pushed, "Mercury pairs pushed");
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
	CAM_INFO(pc, "camera pipeline up%s", pc->hands ? " with Mercury hand tracking" : "");
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
