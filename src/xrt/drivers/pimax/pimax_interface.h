// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to the Pimax Dream Air driver.
 * @ingroup drv_pimax
 */

#pragma once

#include "xrt/xrt_prober.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * @defgroup drv_pimax Pimax Dream Air driver
 * @ingroup drv
 *
 * @brief 3DoF driver for the Pimax Dream Air on top of libpimax (hidraw wake/keepalive + 1 kHz IMU).
 *
 * Cameras / hands (pimax_camera.h): PIMAX_HANDS=mercury, PIMAX_CAM=1, PIMAX_CAM_SOURCE, PIMAX_CAM_RECORD,
 * PIMAX_CAM_CALIB, PIMAX_HT_CAMS, PIMAX_HT_ORIENT, PIMAX_HT_HEAD_OFFSET, PIMAX_HT_STATS.
 *
 * Env vars: PIMAX_LOG, PIMAX_PANEL_ROTATION (0/90/180/270, default 0, TODO),
 * PIMAX_AXIS_MODE (0 = identity, 1 = 180 deg about Y, default: verified by nose-down tilt test 2026-10-13), PIMAX_BIAS_SECONDS (default 2).
 */

#define PIMAX_DRV_VID 0x34a4
#define PIMAX_DRV_PID 0x0044

/*! Prober "found" function, matches USB VID/PID. */
int
pimax_found(struct xrt_prober *xp,
            struct xrt_prober_device **devices,
            size_t device_count,
            size_t index,
            cJSON *attached_data,
            struct xrt_device **out_xdev);

/*! Create the HMD (opens libpimax session). out_ctrls[2] receives left/right controllers (NULL if none). */
struct xrt_device *
pimax_hmd_create(struct xrt_device *out_ctrls[2]);

/*! Mercury hand-tracking device of the HMD's camera pipeline (PIMAX_HANDS=mercury), or NULL. Call once; the caller
 * (the system) owns it afterwards. */
struct xrt_device *
pimax_hmd_take_hand_tracker(struct xrt_device *hmd);

#ifdef __cplusplus
}
#endif
