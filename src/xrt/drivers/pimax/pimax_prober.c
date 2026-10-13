// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air prober (USB VID/PID 34a4:0044).
 * @ingroup drv_pimax
 */

#include "util/u_logging.h"
#include "pimax_interface.h"

int
pimax_found(struct xrt_prober *xp,
            struct xrt_prober_device **devices,
            size_t device_count,
            size_t index,
            cJSON *attached_data,
            struct xrt_device **out_xdev)
{
	// The device can show up more than once (one entry per USB interface); only create one HMD.
	static bool created = false;
	if (created) {
		return 0;
	}

	struct xrt_device *ctrls[2] = {NULL, NULL};
	struct xrt_device *xdev = pimax_hmd_create(ctrls);
	if (xdev == NULL) {
		return -1;
	}
	created = true;
	out_xdev[0] = xdev;
	int n = 1;
	for (int i = 0; i < 2; i++) {
		if (ctrls[i] != NULL) {
			out_xdev[n++] = ctrls[i];
		}
	}
	struct xrt_device *hands = pimax_hmd_take_hand_tracker(xdev);
	if (hands != NULL) {
		out_xdev[n++] = hands;
	}
	return n;
}
