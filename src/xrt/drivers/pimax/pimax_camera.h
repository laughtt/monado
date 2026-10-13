// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air tracking cameras: capture (Rust pimax-cam) into xrt_frame sinks, Mercury hand tracking.
 * @ingroup drv_pimax
 */

#pragma once

#include "xrt/xrt_device.h"
#include "util/u_logging.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * IMU clock reference published by the HMD: monotonic_ns = dev_ext_us * 1000 + offset_ns for the IMU sample whose
 * u32 device timestamp is dev_ts_u32. Same layout as pimax_cam_imu_clock_ref.
 */
struct pimax_camera_clock_ref
{
	uint32_t dev_ts_u32;
	uint32_t valid;
	uint64_t dev_ext_us;
	int64_t offset_ns;
};

typedef bool (*pimax_camera_clock_fn)(void *user, struct pimax_camera_clock_ref *out);

struct pimax_camera;

/*!
 * Create the camera pipeline when enabled by the environment, else return NULL:
 *  - PIMAX_HANDS=mercury: Mercury hand tracking on calibration cameras PIMAX_HT_CAMS (default "0,1" = trackingA,
 *    trackingB), exposed as a hand-tracking device (XR_EXT_hand_tracking) attached to @p hmd.
 *  - PIMAX_CAM=1: capture only (debug sinks for the debug GUI, rate log).
 *  - PIMAX_CAM_SOURCE: "" = find the camera (default), /dev/videoN, or a .pxcam recording (replayed in real time,
 *    looping) for tests without the headset.
 *  - PIMAX_CAM_RECORD=<file.pxcam>: also record every raw frame.
 *  - PIMAX_CAM_CALIB=<xml>: factory calibration (default search, see pimax_cam.h).
 *  - PIMAX_HT_HEAD_OFFSET="x,y,z": head origin in the IMU body frame in metres (default: between the two tracking
 *    cameras, 5 cm behind them).
 *  - PIMAX_HT_STATS=1: log Mercury's processed-pair rate (needs HTA_PREDICTION_DISABLE=1; test aid).
 *
 * @p clock (may be NULL) gives the HMD's IMU clock so frames share the IMU timeline.
 */
struct pimax_camera *
pimax_camera_create(struct xrt_device *hmd,
                    enum u_logging_level log_level,
                    pimax_camera_clock_fn clock,
                    void *clock_user);

/*!
 * The hand-tracking device (attached to the HMD pose), or NULL. Ownership moves to the caller (the system destroys
 * it); call at most once.
 */
struct xrt_device *
pimax_camera_take_hand_device(struct pimax_camera *pc);

/*! Stop capture and free everything (frame graph included). The HMD must outlive this call. */
void
pimax_camera_destroy(struct pimax_camera **pc_ptr);

#ifdef __cplusplus
}
#endif
