// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air lens model. See analysis/lens-distortion.md in the pimax-re tree.
 *
 * Model (per eye, per colour channel c): for a point (u, v) of the user-facing display (after the compositor's
 * panel pre-rotation; v = 0 is the top), p = ((u-.5)*2*aspect, (.5-v)*2) - centre, in units of half the display
 * height. r2 = |p/ca_c|^2, ray tangent t = base*scale*(p/ca_c)*(1 + k1 r2 + k2 r2^2 + k3 r2^3). The client image
 * covers the configured FOV in tangent space, so the sampled texture coordinate is the linear map of t.
 *
 * @ingroup drv_pimax
 */

#include "pimax_lens.h"

#include "util/u_logging.h"
#include "math/m_mathinclude.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEG2RAD(x) ((x) * (float)(M_PI / 180.0))

/*
 * Defaults: fitted (least squares on angle, max error 0.28 deg) to the Dream Air "like Pimax" angle->panel-radius
 * table from analysis/customheadsetopenvr.md; base 0.653 tan/unit. Chromatic aberration from the red/blue offset
 * tables of the same document (+0.45 % red, -0.5 % blue radial). Optical centre 2.7 % toward the nose, 0.9 % up.
 */
static void
set_defaults(struct pimax_lens *lens, float aspect)
{
	lens->enabled = true;
	lens->aspect = aspect;
	for (int e = 0; e < 2; e++) {
		struct pimax_lens_eye *l = &lens->eye[e];
		l->fov_left = l->fov_right = 50.0f;
		l->fov_up = l->fov_down = 44.5f;
		l->cx = e == 0 ? 0.0271f : -0.0271f; // left eye: nose is to the right
		l->cy = 0.009f;
		l->base = 0.65307f;
		l->scale = 1.0f;
		for (int c = 0; c < 3; c++) {
			l->k[c][0] = 0.60779f;
			l->k[c][1] = -0.72489f;
			l->k[c][2] = 0.58858f;
		}
		l->ca[0] = 1.0045f;
		l->ca[1] = 1.0f;
		l->ca[2] = 0.995f;
	}
}

static char *
trim(char *s)
{
	while (isspace((unsigned char)*s)) {
		s++;
	}
	char *e = s + strlen(s);
	while (e > s && isspace((unsigned char)e[-1])) {
		*--e = 0;
	}
	return s;
}

static bool
parse_floats(const char *s, float *out, int n)
{
	char *end;
	for (int i = 0; i < n; i++) {
		out[i] = strtof(s, &end);
		if (end == s) {
			return false;
		}
		s = end;
		while (*s == ',' || isspace((unsigned char)*s)) {
			s++;
		}
	}
	return true;
}

static bool
apply_eye_key(struct pimax_lens_eye *e, const char *key, const char *val)
{
	float f[4];
	if (strcmp(key, "fov") == 0 && parse_floats(val, f, 4)) {
		e->fov_left = f[0];
		e->fov_right = f[1];
		e->fov_up = f[2];
		e->fov_down = f[3];
	} else if (strcmp(key, "center") == 0 && parse_floats(val, f, 2)) {
		e->cx = f[0];
		e->cy = f[1];
	} else if (strcmp(key, "base") == 0 && parse_floats(val, f, 1)) {
		e->base = f[0];
	} else if (strcmp(key, "scale") == 0 && parse_floats(val, f, 1)) {
		e->scale = f[0];
	} else if ((strcmp(key, "k") == 0 || strcmp(key, "k_r") == 0 || strcmp(key, "k_g") == 0 ||
	            strcmp(key, "k_b") == 0) &&
	           parse_floats(val, f, 3)) {
		for (int c = 0; c < 3; c++) {
			if (key[1] == 0 || "rgb"[c] == key[2]) {
				memcpy(e->k[c], f, sizeof(float) * 3);
			}
		}
	} else if (strcmp(key, "ca_r") == 0 && parse_floats(val, f, 1)) {
		e->ca[0] = f[0];
	} else if (strcmp(key, "ca_g") == 0 && parse_floats(val, f, 1)) {
		e->ca[1] = f[0];
	} else if (strcmp(key, "ca_b") == 0 && parse_floats(val, f, 1)) {
		e->ca[2] = f[0];
	} else {
		return false;
	}
	return true;
}

static void
load_file(struct pimax_lens *lens, const char *path)
{
	FILE *fp = fopen(path, "r");
	if (fp == NULL) {
		U_LOG_I("pimax lens: no config at %s, using built-in defaults", path);
		return;
	}
	U_LOG_I("pimax lens: reading %s", path);
	char line[512];
	int section = 0; // 0 = common (both eyes), 1 = left, 2 = right
	int lineno = 0;
	while (fgets(line, sizeof(line), fp) != NULL) {
		lineno++;
		char *h = strpbrk(line, "#;");
		if (h != NULL) {
			*h = 0;
		}
		char *s = trim(line);
		if (*s == 0) {
			continue;
		}
		if (*s == '[') {
			if (strncmp(s, "[left]", 6) == 0) {
				section = 1;
			} else if (strncmp(s, "[right]", 7) == 0) {
				section = 2;
			} else if (strncmp(s, "[common]", 8) == 0 || strncmp(s, "[both]", 6) == 0) {
				section = 0;
			} else {
				U_LOG_W("pimax lens: %s:%d unknown section %s", path, lineno, s);
			}
			continue;
		}
		char *eq = strchr(s, '=');
		if (eq == NULL) {
			U_LOG_W("pimax lens: %s:%d expected key = value", path, lineno);
			continue;
		}
		*eq = 0;
		char *key = trim(s), *val = trim(eq + 1);
		for (char *p = key; *p; p++) {
			*p = (char)tolower((unsigned char)*p);
		}
		if (strcmp(key, "distortion") == 0) {
			lens->enabled = !(strcmp(val, "off") == 0 || strcmp(val, "0") == 0 || strcmp(val, "false") == 0 ||
			                  strcmp(val, "no") == 0);
			continue;
		}
		bool ok = true;
		for (int e = 0; e < 2; e++) {
			if (section == 0 || section == e + 1) {
				ok = apply_eye_key(&lens->eye[e], key, val) && ok;
			}
		}
		if (!ok) {
			U_LOG_W("pimax lens: %s:%d bad key/value '%s = %s'", path, lineno, key, val);
		}
	}
	fclose(fp);
}

void
pimax_lens_load(struct pimax_lens *lens, float aspect)
{
	set_defaults(lens, aspect);

	char path[512];
	const char *over = getenv("PIMAX_LENS_CONF");
	if (over != NULL && *over != 0) {
		snprintf(path, sizeof(path), "%s", over);
	} else {
		const char *xdg = getenv("XDG_CONFIG_HOME");
		const char *home = getenv("HOME");
		if (xdg != NULL && *xdg != 0) {
			snprintf(path, sizeof(path), "%s/pimax/lens.conf", xdg);
		} else {
			snprintf(path, sizeof(path), "%s/.config/pimax/lens.conf", home != NULL ? home : ".");
		}
	}
	load_file(lens, path);

	const char *off = getenv("PIMAX_LENS_OFF");
	if (off != NULL && *off != 0 && *off != '0') {
		lens->enabled = false;
	}

	for (int e = 0; e < 2; e++) {
		const struct pimax_lens_eye *l = &lens->eye[e];
		U_LOG_I("pimax lens: eye %d %s fov L%.1f R%.1f U%.1f D%.1f centre %.4f,%.4f base %.4f scale %.3f", e,
		        lens->enabled ? "distortion on" : "distortion OFF", l->fov_left, l->fov_right, l->fov_up,
		        l->fov_down, l->cx, l->cy, l->base, l->scale);
		U_LOG_I("pimax lens: eye %d k_r %.4f %.4f %.4f  k_g %.4f %.4f %.4f  k_b %.4f %.4f %.4f  ca %.4f %.4f %.4f",
		        e, l->k[0][0], l->k[0][1], l->k[0][2], l->k[1][0], l->k[1][1], l->k[1][2], l->k[2][0],
		        l->k[2][1], l->k[2][2], l->ca[0], l->ca[1], l->ca[2]);
	}
}

struct xrt_fov
pimax_lens_get_fov(const struct pimax_lens *lens, uint32_t view)
{
	const struct pimax_lens_eye *l = &lens->eye[view];
	struct xrt_fov fov = {
	    .angle_left = -DEG2RAD(l->fov_left),
	    .angle_right = DEG2RAD(l->fov_right),
	    .angle_up = DEG2RAD(l->fov_up),
	    .angle_down = -DEG2RAD(l->fov_down),
	};
	return fov;
}

xrt_result_t
pimax_lens_compute(const struct pimax_lens *lens, uint32_t view, float u, float v, struct xrt_uv_triplet *result)
{
	if (!lens->enabled) {
		result->r.x = result->g.x = result->b.x = u;
		result->r.y = result->g.y = result->b.y = v;
		return XRT_SUCCESS;
	}
	const struct pimax_lens_eye *l = &lens->eye[view];
	const float tl = tanf(DEG2RAD(l->fov_left)), tr = tanf(DEG2RAD(l->fov_right));
	const float tu = tanf(DEG2RAD(l->fov_up)), td = tanf(DEG2RAD(l->fov_down));

	const float px = (u - 0.5f) * 2.0f * lens->aspect - l->cx;
	const float py = (0.5f - v) * 2.0f - l->cy;

	struct xrt_vec2 out[3];
	for (int c = 0; c < 3; c++) {
		const float x = px / l->ca[c], y = py / l->ca[c];
		const float r2 = x * x + y * y;
		const float d = l->base * l->scale * (1.0f + r2 * (l->k[c][0] + r2 * (l->k[c][1] + r2 * l->k[c][2])));
		const float tx = x * d, ty = y * d; // ray tangents, y up
		out[c].x = (tx + tl) / (tl + tr);
		out[c].y = (tu - ty) / (tu + td);
	}
	result->r = out[0];
	result->g = out[1];
	result->b = out[2];
	return XRT_SUCCESS;
}
