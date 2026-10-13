// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air lens model: config-file driven radial distortion with per-channel coefficients.
 * @ingroup drv_pimax
 */
#pragma once

#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"

#ifdef __cplusplus
extern "C" {
#endif

struct pimax_lens_eye
{
	//! Half-angles in degrees of the rendered (client) FOV, all positive.
	float fov_left, fov_right, fov_up, fov_down;
	//! Optical centre on the panel, in units of half the display height, +x right, +y up (as the client sees it).
	float cx, cy;
	//! Tangent of the ray angle per unit panel radius at the centre (pinhole focal term).
	float base;
	//! Overall multiplier on ray tangents (>1 shows a wider scene / image looks smaller).
	float scale;
	//! Polynomial k1..k3 per colour channel (0 = R, 1 = G, 2 = B): tan = base*scale*r*(1+k1 r^2+k2 r^4+k3 r^6).
	float k[3][3];
	//! Per-channel radial magnification (lateral chromatic aberration), r is divided by this.
	float ca[3];
};

struct pimax_lens
{
	bool enabled; //!< false: identity mesh (no distortion), FOV from config still applies.
	float aspect; //!< Display width / height as the client sees it (3840/3552).
	struct pimax_lens_eye eye[2];
};

//! Fill defaults, then override from the config file (PIMAX_LENS_CONF or ~/.config/pimax/lens.conf).
void
pimax_lens_load(struct pimax_lens *lens, float aspect);

//! Convert the eye FOV to an xrt_fov.
struct xrt_fov
pimax_lens_get_fov(const struct pimax_lens *lens, uint32_t view);

xrt_result_t
pimax_lens_compute(const struct pimax_lens *lens, uint32_t view, float u, float v, struct xrt_uv_triplet *result);

#ifdef __cplusplus
}
#endif
