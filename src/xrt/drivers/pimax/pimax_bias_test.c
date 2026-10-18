// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Test of the gyro bias rest rule (pimax_bias.h), standalone:
 *
 *     cc -O2 -Wall -o /tmp/pimax_bias_test src/xrt/drivers/pimax/pimax_bias_test.c -lm
 *     /tmp/pimax_bias_test                      # synthetic cases
 *     /tmp/pimax_bias_test head-imu.csv T_WORN T_DESK0 T_DESK1 [prev_x prev_y prev_z]
 *
 * With a session's head-imu.csv (IMU frame, axis mode 1, rad/s and m/s^2, CLOCK_MONOTONIC ns) it feeds every sample
 * through the driver's rule exactly as pimax_hmd.c does (converted back to the raw deg/s and g the driver gets) and
 * through the old rule, prints every completed window and checks: the new rule accepts nothing before T_WORN
 * (monotonic s; the headset was worn) and accepts a window inside [T_DESK0, T_DESK1] (it rested on the desk).
 *
 * @ingroup drv_pimax
 */

#include "pimax_bias.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int failures = 0;

#define CHECK(c, ...)                                                                                                  \
	do {                                                                                                           \
		if (!(c)) {                                                                                            \
			failures++;                                                                                    \
			printf("FAIL: " __VA_ARGS__);                                                                  \
			printf("\n");                                                                                  \
		}                                                                                                      \
	} while (0)

//! Deterministic standard normal (Irwin-Hall from an LCG).
static double
gauss(uint64_t *s)
{
	double x = 0;
	for (int i = 0; i < 12; i++) {
		*s = *s * 6364136223846793005ULL + 1442695040888963407ULL;
		x += (double)(*s >> 11) / (double)(1ULL << 53);
	}
	return x - 6.0;
}

//! Feed `secs` of synthetic samples; returns the last completed-window result.
static enum pimax_bias_result
run_synth(struct pimax_bias_state *st,
          const struct pimax_bias_rule *r,
          const float bias[3],
          double gyro_sd,
          double accel_sd_g,
          double secs,
          float out[3])
{
	uint64_t seed = 7;
	enum pimax_bias_result last = PIMAX_BIAS_MORE;
	for (int k = 0; k < (int)(secs * 1000); k++) {
		float g[3], a[3], og[3], oa[3];
		for (int i = 0; i < 3; i++) {
			g[i] = bias[i] + (float)(gyro_sd * gauss(&seed));
			a[i] = (i == 1 ? 1.0f : 0.0f) + (float)(accel_sd_g * gauss(&seed));
		}
		enum pimax_bias_result res = pimax_bias_feed(st, r, g, a, og, oa);
		if (res != PIMAX_BIAS_MORE) {
			last = res;
			if (res == PIMAX_BIAS_ACCEPT) {
				memcpy(out, og, sizeof(og));
			}
		}
	}
	return last;
}

static void
synthetic(void)
{
	struct pimax_bias_rule r;
	pimax_bias_rule_default(&r, 2000);
	const float bias[3] = {0.118f, 0.775f, 0.370f};
	float out[3] = {0};

	struct pimax_bias_state st = {0};
	CHECK(run_synth(&st, &r, bias, 0.06, 0.0013, 2.1, out) == PIMAX_BIAS_ACCEPT, "a desk rest is accepted");
	CHECK(fabsf(out[1] - 0.775f) < 0.01f, "mean %.3f", out[1]);

	struct pimax_bias_state worn = {0};
	CHECK(run_synth(&worn, &r, bias, 0.5, 0.005, 2.1, out) == PIMAX_BIAS_NOT_REST, "a worn head (0.5 deg/s) is not");
	struct pimax_bias_state jiggle = {0};
	CHECK(run_synth(&jiggle, &r, bias, 0.06, 0.006, 2.1, out) == PIMAX_BIAS_NOT_REST,
	      "a quiet gyro with a shaking accelerometer is not");

	// Far from the bias in use: needs a second window that agrees.
	struct pimax_bias_state far = {0};
	const float wrong[3] = {0.118f, 1.3f, 0.370f};
	pimax_bias_set_prev(&far, wrong);
	CHECK(run_synth(&far, &r, bias, 0.06, 0.0013, 2.1, out) == PIMAX_BIAS_FAR, "first far window waits");
	CHECK(run_synth(&far, &r, bias, 0.06, 0.0013, 2.0, out) == PIMAX_BIAS_ACCEPT, "a confirmed far rest is taken");
	CHECK(fabsf(out[1] - 0.775f) < 0.01f, "confirmed mean %.3f", out[1]);
	printf("synthetic cases: %s\n", failures ? "FAILED" : "ok");
}

static int
replay(int argc, char **argv)
{
	FILE *f = fopen(argv[1], "r");
	if (f == NULL) {
		perror(argv[1]);
		return 2;
	}
	double t_worn = atof(argv[2]), t0 = atof(argv[3]), t1 = atof(argv[4]);
	struct pimax_bias_rule rule, old;
	pimax_bias_rule_default(&rule, 2000);
	// The rule before 2026-10-18: motion and half-means only, any mean.
	old = rule;
	old.gyro_sd_max_dps = 1e9;
	old.accel_sd_max_g = 1e9;
	old.mean_tol_dps = 1e9;
	struct pimax_bias_state sn = {0}, so = {0};
	if (argc >= 8) {
		float p[3] = {(float)atof(argv[5]), (float)atof(argv[6]), (float)atof(argv[7])};
		pimax_bias_set_prev(&sn, p);
	}
	char line[256];
	int new_acc_worn = 0, new_acc_desk = 0, old_acc_worn = 0, rejected[6] = {0};
	float desk_mean[3] = {0};
	while (fgets(line, sizeof(line), f) != NULL) {
		long long ts;
		double ax, ay, az, gx, gy, gz;
		if (line[0] != 'I' || sscanf(line, "I,%lld,%lf,%lf,%lf,%lf,%lf,%lf", &ts, &ax, &ay, &az, &gx, &gy, &gz) != 7) {
			continue;
		}
		double t = ts / 1e9;
		const double D = 180.0 / M_PI, G = 9.80665;
		// IMU frame (axis mode 1: x and z flipped) back to the raw device frame the driver gets.
		float g[3] = {(float)(-gx * D), (float)(gy * D), (float)(-gz * D)};
		float a[3] = {(float)(-ax / G), (float)(ay / G), (float)(-az / G)};
		float og[3], oa[3];
		enum pimax_bias_result rn = pimax_bias_feed(&sn, &rule, g, a, og, oa);
		if (rn != PIMAX_BIAS_MORE && rn != PIMAX_BIAS_MOVING) {
			rejected[rn]++;
			if (rn == PIMAX_BIAS_ACCEPT) {
				if (t < t_worn) {
					new_acc_worn++;
				}
				if (t >= t0 && t <= t1) {
					new_acc_desk++;
					memcpy(desk_mean, og, sizeof(og));
				}
			}
			printf("%.1f new: %-30s mean %+.3f %+.3f %+.3f dps, gyro sd %.3f %.3f %.3f dps, accel sd %.4f %.4f "
			       "%.4f m/s^2\n",
			       t, pimax_bias_result_str(rn), sn.last_mean[0], sn.last_mean[1], sn.last_mean[2], sn.last_gsd[0],
			       sn.last_gsd[1], sn.last_gsd[2], sn.last_asd[0] * G, sn.last_asd[1] * G, sn.last_asd[2] * G);
		}
		enum pimax_bias_result ro = pimax_bias_feed(&so, &old, g, a, og, oa);
		if (ro == PIMAX_BIAS_ACCEPT) {
			if (t < t_worn) {
				old_acc_worn++;
			}
			printf("%.1f old rule accepted: mean %+.3f %+.3f %+.3f dps (gyro sd %.3f %.3f %.3f)\n", t,
			       so.last_mean[0], so.last_mean[1], so.last_mean[2], so.last_gsd[0], so.last_gsd[1],
			       so.last_gsd[2]);
		}
	}
	fclose(f);
	printf("new rule: %d windows accepted while worn, %d on the desk (last %+.3f %+.3f %+.3f dps); rejected: %d not "
	       "a rest, %d drifting, %d far; old rule: %d accepted while worn\n",
	       new_acc_worn, new_acc_desk, desk_mean[0], desk_mean[1], desk_mean[2], rejected[PIMAX_BIAS_NOT_REST],
	       rejected[PIMAX_BIAS_DRIFTING], rejected[PIMAX_BIAS_FAR], old_acc_worn);
	CHECK(new_acc_worn == 0, "a worn window was accepted");
	CHECK(new_acc_desk > 0, "the desk rest was not accepted");
	CHECK(old_acc_worn > 0, "the old rule should have taken a worn window (the 11:59:47 bug)");
	return failures ? 1 : 0;
}

int
main(int argc, char **argv)
{
	synthetic();
	if (argc >= 5) {
		replay(argc, argv);
	}
	printf("%s\n", failures ? "FAILED" : "PASSED");
	return failures ? 1 : 0;
}
