// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Pimax Dream Air controllers. See analysis/monado-controllers.md.
 *
 * Two xrt_devices (left slot 2, right slot 3) exposed with the Oculus Touch profile. No optical tracking:
 * PIMAX_CTRL_POSE=fixed (default): fixed offset from the head, following head yaw only, forward orientation.
 * PIMAX_CTRL_POSE=imu: same position, orientation from the controller IMU (accel 1024 LSB/g, gyro scale is a guess,
 * PIMAX_CTRL_GYRO_LSB_DPS, default 16.4, axes assumed == OpenXR axes).
 *
 * Hand tracking (XR_EXT_hand_tracking, data source CONTROLLER): the finger curls come from the Rust crate pimax-hands
 * (trigger -> index, grip -> middle/ring/little, touch bits -> thumb; filters + thumb state machine live there), this
 * file only feeds it the decoded reports and runs Monado's u_hand_sim_simulate_generic on the result. The hand is
 * posed from the controller's grip pose through a fixed grip->wrist offset (also from pimax-hands).
 * PIMAX_CTRL_HANDS=0 turns it off, PIMAX_CTRL_HANDS_DEMO=1 replaces the reports by a scripted pattern (test aid).
 * The reports are only queued (plain C, a small ring) on the libpimax session thread; the Rust filters run when a
 * client asks for the hand joints (get_hand_tracking), so a session without hand tracking does no hand work at all
 * (hang when worn, analysis/hang-worn-debug.md). PIMAX_CTRL_HANDS_FEED=report restores the old per-report feed on the
 * session thread (A/B test only).
 *
 * @ingroup drv_pimax
 */

#include "os/os_time.h"
#include "os/os_threading.h"
#include "xrt/xrt_defines.h"
#include "xrt/xrt_device.h"
#include "xrt/xrt_config_have.h"

#include "math/m_api.h"
#include "math/m_imu_3dof.h"
#include "math/m_vec3.h"
#include "math/m_mathinclude.h"
#include "math/m_space.h"

#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_hand_simulation.h"
#include "util/u_logging.h"
#include "util/u_misc.h"
#include "util/u_var.h"

#include "pimax_ctrl.h"
#include "pimax_hands.h"

#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

DEBUG_GET_ONCE_OPTION(pimax_ctrl_pose, "PIMAX_CTRL_POSE", "fixed")
DEBUG_GET_ONCE_FLOAT_OPTION(pimax_ctrl_gyro_lsb_dps, "PIMAX_CTRL_GYRO_LSB_DPS", 16.4f)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ctrl_hands, "PIMAX_CTRL_HANDS", true)
DEBUG_GET_ONCE_BOOL_OPTION(pimax_ctrl_hands_demo, "PIMAX_CTRL_HANDS_DEMO", false)
DEBUG_GET_ONCE_OPTION(pimax_ctrl_hands_feed, "PIMAX_CTRL_HANDS_FEED", "lazy")

// Fixed offsets in the head-yaw frame (x right, y up, -z forward).
#define OFF_SIDE 0.20f
#define OFF_DOWN 0.30f
#define OFF_FWD 0.35f

#define ACTIVE_TIMEOUT_NS (1500ull * 1000 * 1000) // no report for this long -> inputs inactive
#define REATTACH_MIN_NS (2500ull * 1000 * 1000)
#define BIAS_SAMPLES 250 // ~1 s at 250 Hz
#define HANDS_RING 128   // queued reports for the lazy hand feed: ~0.5 s at 250 Hz (older ones are dropped)

enum
{
	I_BTN1_CLICK, // left X / right A  (report A bit)
	I_BTN1_TOUCH,
	I_BTN2_CLICK, // left Y / right B  (report B bit)
	I_BTN2_TOUCH,
	I_MENU, // left MENU_CLICK (confirmed); right SYSTEM_CLICK (unverified, report menu bit)
	I_SQUEEZE,
	I_TRIGGER_TOUCH,
	I_TRIGGER_VALUE,
	I_STICK_CLICK,
	I_STICK_TOUCH,
	I_STICK,
	I_GRIP_POSE,
	I_AIM_POSE,
	I_HAND_TRACKING, // XRT_INPUT_HT_CONFORMING_*; last, only allocated when hand tracking is on
	I_COUNT,
};

struct pimax_ctrl_ctx
{
	struct xrt_reference ref;
	struct xrt_device *head;
	enum u_logging_level log_level;
	bool imu_mode;
	float gyro_scale; // rad/s per LSB
	bool hands;       // controller-driven hand tracking (pimax-hands)
	bool hands_demo;  // scripted input instead of reports (PIMAX_CTRL_HANDS_DEMO)
	bool hands_eager; // PIMAX_CTRL_HANDS_FEED=report: feed Rust on the session thread (old behaviour)

	struct pimax_controller *ctl[2]; // [0] left (slot 2), [1] right (slot 3)

	// Attach thread.
	pimax_t *lib;
	pthread_t thread;
	bool thread_running;
	atomic_bool stop;
	atomic_bool need_attach_all; // set on (re)connect
};

struct pimax_controller
{
	struct xrt_device base;
	struct pimax_ctrl_ctx *ctx;
	int index; // 0 left, 1 right
	uint8_t slot;

	pthread_mutex_t lock;
	// Latest state (lock).
	pimax_controller_report rep;
	bool have_report;
	uint64_t last_report_ns;
	uint32_t reports;
	float trigger, grip, stick_x, stick_y;
	bool btn_a, btn_b, btn_menu, btn_stick, t_a, t_b, t_stick;
	uint8_t raw46;

	// Hand tracking (lock): Rust state (curl filters, thumb state machine) and its latest result.
	pimax_hands *hands;
	pimax_hands_out hands_out;
	struct xrt_pose grip_to_wrist; // wrist in the grip pose frame
	// Lazy feed (lock): reports queued by the session thread, drained into pimax-hands by get_hand_tracking.
	struct
	{
		uint64_t ns;
		uint32_t buttons;
		int16_t stick_x, stick_y;
		uint8_t trigger, grip, touch;
	} hq[HANDS_RING];
	uint32_t hq_head;  // next write index
	uint32_t hq_count; // queued, <= HANDS_RING

	// IMU orientation (lock).
	struct m_imu_3dof fusion;
	bool fusion_init;
	bool bias_done;
	uint32_t bias_n;
	double gyro_sum[3], acc_sum[3];
	struct xrt_quat yaw_ref; // head yaw when the IMU tilt was seeded
	struct xrt_quat imu_rot;

	// Attach logic (written from the session thread and the attach thread, atomics).
	_Atomic uint64_t last_status_ns;
	_Atomic uint64_t last_attach_ns;
	atomic_uint attaches;
};

static inline struct pimax_controller *
ctl(struct xrt_device *xdev)
{
	return (struct pimax_controller *)xdev;
}

#define C_INFO(c, ...) U_LOG_XDEV_IFL_I(&(c)->base, (c)->ctx->log_level, __VA_ARGS__)
#define C_WARN(c, ...) U_LOG_XDEV_IFL_W(&(c)->base, (c)->ctx->log_level, __VA_ARGS__)

/*
 * Binding profiles (khr/simple_controller and Valve Index emulate this device).
 */
static struct xrt_binding_input_pair simple_inputs_left[4] = {
    {XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_SIMPLE_MENU_CLICK, XRT_INPUT_TOUCH_MENU_CLICK},
    {XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
    {XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE},
};
static struct xrt_binding_input_pair simple_inputs_right[4] = {
    {XRT_INPUT_SIMPLE_SELECT_CLICK, XRT_INPUT_TOUCH_TRIGGER_VALUE},
    {XRT_INPUT_SIMPLE_MENU_CLICK, XRT_INPUT_TOUCH_B_CLICK},
    {XRT_INPUT_SIMPLE_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE},
    {XRT_INPUT_SIMPLE_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE},
};
static struct xrt_binding_output_pair simple_outputs[1] = {
    {XRT_OUTPUT_NAME_SIMPLE_VIBRATION, XRT_OUTPUT_NAME_TOUCH_HAPTIC},
};

#define INDEX_COMMON                                                                                                   \
	{XRT_INPUT_INDEX_TRIGGER_VALUE, XRT_INPUT_TOUCH_TRIGGER_VALUE},                                                \
	    {XRT_INPUT_INDEX_TRIGGER_TOUCH, XRT_INPUT_TOUCH_TRIGGER_TOUCH},                                            \
	    {XRT_INPUT_INDEX_SQUEEZE_VALUE, XRT_INPUT_TOUCH_SQUEEZE_VALUE},                                            \
	    {XRT_INPUT_INDEX_THUMBSTICK, XRT_INPUT_TOUCH_THUMBSTICK},                                                  \
	    {XRT_INPUT_INDEX_THUMBSTICK_CLICK, XRT_INPUT_TOUCH_THUMBSTICK_CLICK},                                      \
	    {XRT_INPUT_INDEX_THUMBSTICK_TOUCH, XRT_INPUT_TOUCH_THUMBSTICK_TOUCH},                                      \
	    {XRT_INPUT_INDEX_GRIP_POSE, XRT_INPUT_TOUCH_GRIP_POSE}, {XRT_INPUT_INDEX_AIM_POSE, XRT_INPUT_TOUCH_AIM_POSE}
static struct xrt_binding_input_pair index_inputs_left[10] = {
    INDEX_COMMON,
    {XRT_INPUT_INDEX_A_CLICK, XRT_INPUT_TOUCH_X_CLICK},
    {XRT_INPUT_INDEX_B_CLICK, XRT_INPUT_TOUCH_Y_CLICK},
};
static struct xrt_binding_input_pair index_inputs_right[10] = {
    INDEX_COMMON,
    {XRT_INPUT_INDEX_A_CLICK, XRT_INPUT_TOUCH_A_CLICK},
    {XRT_INPUT_INDEX_B_CLICK, XRT_INPUT_TOUCH_B_CLICK},
};
static struct xrt_binding_output_pair index_outputs[1] = {
    {XRT_OUTPUT_NAME_INDEX_HAPTIC, XRT_OUTPUT_NAME_TOUCH_HAPTIC},
};

static struct xrt_binding_profile profiles_left[2] = {
    {.name = XRT_DEVICE_SIMPLE_CONTROLLER,
     .inputs = simple_inputs_left,
     .input_count = ARRAY_SIZE(simple_inputs_left),
     .outputs = simple_outputs,
     .output_count = ARRAY_SIZE(simple_outputs)},
    {.name = XRT_DEVICE_INDEX_CONTROLLER,
     .inputs = index_inputs_left,
     .input_count = ARRAY_SIZE(index_inputs_left),
     .outputs = index_outputs,
     .output_count = ARRAY_SIZE(index_outputs)},
};
static struct xrt_binding_profile profiles_right[2] = {
    {.name = XRT_DEVICE_SIMPLE_CONTROLLER,
     .inputs = simple_inputs_right,
     .input_count = ARRAY_SIZE(simple_inputs_right),
     .outputs = simple_outputs,
     .output_count = ARRAY_SIZE(simple_outputs)},
    {.name = XRT_DEVICE_INDEX_CONTROLLER,
     .inputs = index_inputs_right,
     .input_count = ARRAY_SIZE(index_inputs_right),
     .outputs = index_outputs,
     .output_count = ARRAY_SIZE(index_outputs)},
};

/*
 * Device functions.
 */

static void
ctx_unref(struct pimax_ctrl_ctx *ctx)
{
	if (xrt_reference_dec_and_is_zero(&ctx->ref)) {
		free(ctx);
	}
}

static void
pimax_ctl_destroy(struct xrt_device *xdev)
{
	struct pimax_controller *c = ctl(xdev);
	struct pimax_ctrl_ctx *ctx = c->ctx;
	ctx->ctl[c->index] = NULL;
	u_var_remove_root(c);
	if (c->fusion_init) {
		m_imu_3dof_close(&c->fusion);
	}
	pthread_mutex_lock(&c->lock);
	pimax_hands *hands = c->hands; // a late report must not find a freed hand state
	c->hands = NULL;
	pthread_mutex_unlock(&c->lock);
	pimax_hands_destroy(hands);
	pthread_mutex_destroy(&c->lock);
	u_device_free(&c->base);
	ctx_unref(ctx);
}

static float
clampf(float v, float lo, float hi)
{
	return v < lo ? lo : (v > hi ? hi : v);
}

static xrt_result_t
pimax_ctl_update_inputs(struct xrt_device *xdev)
{
	struct pimax_controller *c = ctl(xdev);
	bool left = c->index == 0;
	uint64_t now = os_monotonic_get_ns();

	pthread_mutex_lock(&c->lock);
	bool active = c->have_report && (now - c->last_report_ns) < ACTIVE_TIMEOUT_NS;
	uint64_t ts = c->have_report ? (int64_t)c->last_report_ns : (int64_t)now;
	struct xrt_input *in = c->base.inputs;
	in[I_BTN1_CLICK].value.boolean = c->btn_a;
	in[I_BTN1_TOUCH].value.boolean = c->t_a || c->btn_a;
	in[I_BTN2_CLICK].value.boolean = c->btn_b;
	in[I_BTN2_TOUCH].value.boolean = c->t_b || c->btn_b;
	in[I_MENU].value.boolean = c->btn_menu;
	in[I_SQUEEZE].value.vec1.x = c->grip;
	in[I_TRIGGER_VALUE].value.vec1.x = c->trigger;
	in[I_TRIGGER_TOUCH].value.boolean = c->trigger > 0.06f;
	in[I_STICK_CLICK].value.boolean = c->btn_stick;
	in[I_STICK_TOUCH].value.boolean = c->t_stick || c->btn_stick;
	in[I_STICK].value.vec2.x = c->stick_x;
	in[I_STICK].value.vec2.y = c->stick_y;
	pthread_mutex_unlock(&c->lock);
	(void)left;

	for (int i = 0; i < I_GRIP_POSE; i++) {
		in[i].active = active;
		in[i].timestamp = (int64_t)ts;
	}
	// Poses are synthesized (fixed offset from the head), so they are always active.
	in[I_GRIP_POSE].active = in[I_AIM_POSE].active = true;
	in[I_GRIP_POSE].timestamp = in[I_AIM_POSE].timestamp = (int64_t)now;
	return XRT_SUCCESS;
}

//! Yaw-only quaternion of the head orientation (about +Y). Identity if the head has no valid orientation.
static struct xrt_quat
head_yaw(struct pimax_controller *c, int64_t at_ns)
{
	struct xrt_quat q = XRT_QUAT_IDENTITY;
	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	if (c->ctx->head == NULL ||
	    xrt_device_get_tracked_pose(c->ctx->head, XRT_INPUT_GENERIC_HEAD_POSE, at_ns, &rel) != XRT_SUCCESS ||
	    (rel.relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return q;
	}
	struct xrt_vec3 fwd_in = {0, 0, -1}, fwd;
	math_quat_rotate_vec3(&rel.pose.orientation, &fwd_in, &fwd);
	if (fabsf(fwd.x) + fabsf(fwd.z) < 1e-3f) {
		return q; // looking straight up/down: yaw undefined
	}
	float yaw = atan2f(-fwd.x, -fwd.z);
	struct xrt_vec3 up = {0, 1, 0};
	math_quat_from_angle_vector(yaw, &up, &q);
	return q;
}

static xrt_result_t
pimax_ctl_get_tracked_pose(struct xrt_device *xdev,
                           enum xrt_input_name name,
                           int64_t at_timestamp_ns,
                           struct xrt_space_relation *out_relation)
{
	struct pimax_controller *c = ctl(xdev);
	if (name != XRT_INPUT_TOUCH_GRIP_POSE && name != XRT_INPUT_TOUCH_AIM_POSE) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&c->base, c->ctx->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	struct xrt_quat yaw = head_yaw(c, at_timestamp_ns);
	struct xrt_vec3 off = {c->index == 0 ? -OFF_SIDE : OFF_SIDE, -OFF_DOWN, -OFF_FWD};
	struct xrt_vec3 pos;
	math_quat_rotate_vec3(&yaw, &off, &pos);

	struct xrt_quat orient = yaw;
	if (c->ctx->imu_mode) {
		pthread_mutex_lock(&c->lock);
		if (c->bias_done) {
			math_quat_rotate(&c->yaw_ref, &c->imu_rot, &orient);
			math_quat_normalize(&orient);
		}
		pthread_mutex_unlock(&c->lock);
	}

	struct xrt_space_relation rel = XRT_SPACE_RELATION_ZERO;
	rel.pose.orientation = orient;
	rel.pose.position = pos;
	// Position is a fixed guess, not tracked: valid but not "tracked".
	rel.relation_flags = (enum xrt_space_relation_flags)(XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	                                                     XRT_SPACE_RELATION_POSITION_VALID_BIT |
	                                                     XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);
	*out_relation = rel;
	return XRT_SUCCESS;
}

//! Lock held: feed the queued reports (oldest first) into pimax-hands and refresh hands_out.
static void
hands_drain_locked(struct pimax_controller *c)
{
	if (c->hands == NULL || c->hq_count == 0) {
		return;
	}
	uint32_t start = (c->hq_head + HANDS_RING - c->hq_count) % HANDS_RING;
	for (uint32_t k = 0; k < c->hq_count; k++) {
		const uint32_t i = (start + k) % HANDS_RING;
		pimax_hands_update_report(c->hands, c->hq[i].ns, c->hq[i].trigger, c->hq[i].grip, c->hq[i].stick_x,
		                          c->hq[i].stick_y, c->hq[i].buttons, c->hq[i].touch);
	}
	c->hq_count = 0;
	pimax_hands_get(c->hands, &c->hands_out);
}

//! Fill the joint set of this hand from the Rust curl state; the hand follows the grip pose.
static xrt_result_t
pimax_ctl_get_hand_tracking(struct xrt_device *xdev,
                            enum xrt_input_name name,
                            int64_t at_timestamp_ns,
                            struct xrt_hand_joint_set *out_value,
                            int64_t *out_timestamp_ns)
{
	struct pimax_controller *c = ctl(xdev);
	bool left = c->index == 0;
	if (name != (left ? XRT_INPUT_HT_CONFORMING_LEFT : XRT_INPUT_HT_CONFORMING_RIGHT)) {
		U_LOG_XDEV_UNSUPPORTED_INPUT(&c->base, c->ctx->log_level, name);
		return XRT_ERROR_INPUT_UNSUPPORTED;
	}

	uint64_t now = os_monotonic_get_ns();
	pimax_hands_out ho;
	pthread_mutex_lock(&c->lock);
	bool active = c->have_report && (now - c->last_report_ns) < ACTIVE_TIMEOUT_NS;
	if (c->ctx->hands_demo) {
		// Scripted input instead of reports; always "active" so a test needs no awake controller.
		pimax_hands_input demo;
		pimax_hands_demo_input(now, !left, &demo);
		pimax_hands_update(c->hands, now, &demo);
		pimax_hands_get(c->hands, &c->hands_out);
		active = true;
	} else {
		hands_drain_locked(c);
	}
	ho = c->hands_out;
	pthread_mutex_unlock(&c->lock);

	// Wrist = grip pose * fixed offset (the joint set is expressed relative to its hand_pose, in tracking-origin space).
	struct xrt_space_relation grip = XRT_SPACE_RELATION_ZERO;
	xrt_result_t xret = pimax_ctl_get_tracked_pose(xdev, XRT_INPUT_TOUCH_GRIP_POSE, at_timestamp_ns, &grip);
	if (xret != XRT_SUCCESS) {
		return xret;
	}
	struct xrt_relation_chain chain = {0};
	m_relation_chain_push_pose(&chain, &c->grip_to_wrist);
	m_relation_chain_push_relation(&chain, &grip);
	struct xrt_space_relation wrist = XRT_SPACE_RELATION_ZERO;
	m_relation_chain_resolve(&chain, &wrist);

	// Same order as struct u_hand_tracking_values.
	const pimax_hands_finger *src[5] = {&ho.little, &ho.ring, &ho.middle, &ho.index, &ho.thumb};
	struct u_hand_tracking_values values;
	struct u_hand_tracking_finger_value *dst[5] = {&values.little, &values.ring, &values.middle, &values.index,
	                                               &values.thumb};
	for (int f = 0; f < 5; f++) {
		dst[f]->splay = src[f]->splay;
		for (int j = 0; j < 4; j++) {
			dst[f]->joint_curls[j] = src[f]->joint_curls[j];
		}
		dst[f]->joint_count = 4;
	}

	u_hand_sim_simulate_generic(&values, left ? XRT_HAND_LEFT : XRT_HAND_RIGHT, &wrist, out_value);

	// The simulation marks the hand pose fully tracked; keep the real flags of the (synthesized) grip pose.
	out_value->hand_pose = wrist;
	out_value->is_active = active;
	*out_timestamp_ns = at_timestamp_ns;
	return XRT_SUCCESS;
}

static xrt_result_t
pimax_ctl_set_output(struct xrt_device *xdev, enum xrt_output_name name, const struct xrt_output_value *value)
{
	return XRT_SUCCESS; // haptics: protocol unknown, no-op
}

static xrt_result_t
pimax_ctl_get_battery_status(struct xrt_device *xdev, bool *out_present, bool *out_charging, float *out_charge)
{
	struct pimax_controller *c = ctl(xdev);
	pthread_mutex_lock(&c->lock);
	bool have = c->have_report;
	uint8_t b = c->raw46;
	pthread_mutex_unlock(&c->lock);
	*out_present = have;
	*out_charging = (b & 0x80) != 0;
	*out_charge = have ? clampf((float)(b & 0x7f) / 100.0f, 0.0f, 1.0f) : 0.0f;
	return XRT_SUCCESS;
}

/*
 * Reports (session thread).
 */

static void
imu_update(struct pimax_controller *c, const pimax_controller_report *r, uint64_t now)
{
	struct pimax_ctrl_ctx *ctx = c->ctx;
	float a[3], g[3];
	for (int i = 0; i < 3; i++) {
		a[i] = (float)r->raw_accel[i] / 1024.0f; // g
		g[i] = (float)r->raw_gyro[i] * ctx->gyro_scale;
	}

	if (!c->fusion_init) {
		m_imu_3dof_init(&c->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);
		c->fusion_init = true;
	}

	if (!c->bias_done) {
		// Average ~1 s while still (|gyro| below ~12 deg/s sum), else restart.
		float gm = (fabsf(g[0]) + fabsf(g[1]) + fabsf(g[2])) * (float)(180.0 / M_PI);
		if (gm > 12.0f) {
			c->bias_n = 0;
			memset(c->gyro_sum, 0, sizeof(c->gyro_sum));
			memset(c->acc_sum, 0, sizeof(c->acc_sum));
			return;
		}
		for (int i = 0; i < 3; i++) {
			c->gyro_sum[i] += g[i];
			c->acc_sum[i] += a[i];
		}
		if (++c->bias_n < BIAS_SAMPLES) {
			return;
		}
		struct xrt_vec3 gb = {(float)(c->gyro_sum[0] / c->bias_n), (float)(c->gyro_sum[1] / c->bias_n),
		                      (float)(c->gyro_sum[2] / c->bias_n)};
		struct xrt_vec3 ac = {(float)(c->acc_sum[0] / c->bias_n), (float)(c->acc_sum[1] / c->bias_n),
		                      (float)(c->acc_sum[2] / c->bias_n)};
		c->fusion.gyro_bias.value = gb;
		struct xrt_vec3 up = {0, 1, 0};
		struct xrt_vec3 acn = m_vec3_normalize(ac);
		math_quat_from_vec_a_to_vec_b(&acn, &up, &c->fusion.rot);
		math_quat_normalize(&c->fusion.rot);
		c->yaw_ref = head_yaw(c, (int64_t)now);
		c->bias_done = true;
		C_INFO(c, "IMU bias done (gyro %.4f %.4f %.4f rad/s, |acc| %.2f g)", gb.x, gb.y, gb.z,
		       m_vec3_len(ac));
		return;
	}

	struct xrt_vec3 accel = {a[0] * 9.80665f, a[1] * 9.80665f, a[2] * 9.80665f};
	struct xrt_vec3 gyro = {g[0], g[1], g[2]};
	m_imu_3dof_update(&c->fusion, now, &accel, &gyro);
	c->imu_rot = c->fusion.rot;
}

void
pimax_ctrl_on_report(struct pimax_ctrl_ctx *ctx, const pimax_controller_report *r)
{
	if (ctx == NULL || (r->slot != PIMAX_SLOT_LEFT && r->slot != PIMAX_SLOT_RIGHT)) {
		return;
	}
	struct pimax_controller *c = ctx->ctl[r->slot == PIMAX_SLOT_LEFT ? 0 : 1];
	if (c == NULL) {
		return;
	}
	uint64_t now = os_monotonic_get_ns();
	pthread_mutex_lock(&c->lock);
	if (!c->have_report) {
		C_INFO(c, "first report (slot %u)", r->slot);
	}
	c->have_report = true;
	c->last_report_ns = now;
	c->reports++;
	c->rep = *r;
	c->trigger = (float)r->trigger / 255.0f;
	c->grip = (float)r->grip / 255.0f;
	c->stick_x = clampf((float)r->stick_x / 127.0f, -1.0f, 1.0f);
	c->stick_y = clampf((float)r->stick_y / 127.0f, -1.0f, 1.0f);
	c->btn_a = (r->buttons & PIMAX_BTN_A) != 0;
	c->btn_b = (r->buttons & PIMAX_BTN_B) != 0;
	c->btn_menu = (r->buttons & PIMAX_BTN_MENU) != 0;
	c->btn_stick = (r->buttons & PIMAX_BTN_STICK) != 0;
	c->t_a = (r->touch & 0x01) != 0;
	c->t_b = (r->touch & 0x02) != 0;
	c->t_stick = (r->touch & PIMAX_TOUCH_STICK) != 0;
	c->raw46 = r->raw[46];
	if (c->hands != NULL && !ctx->hands_demo) {
		// Bit meanings (touch, buttons), normalisation, filters and the thumb state machine are all in Rust. Only
		// queue the report here (session thread); get_hand_tracking runs the filters when a client wants hands.
		const uint32_t i = c->hq_head;
		c->hq[i].ns = now;
		c->hq[i].buttons = r->buttons;
		c->hq[i].stick_x = (int16_t)r->stick_x;
		c->hq[i].stick_y = (int16_t)r->stick_y;
		c->hq[i].trigger = r->trigger;
		c->hq[i].grip = r->grip;
		c->hq[i].touch = r->touch;
		c->hq_head = (i + 1) % HANDS_RING;
		if (c->hq_count < HANDS_RING) {
			c->hq_count++;
		}
		if (ctx->hands_eager) {
			hands_drain_locked(c);
		}
	}
	if (ctx->imu_mode) {
		imu_update(c, r, now);
	}
	pthread_mutex_unlock(&c->lock);
}

void
pimax_ctrl_on_status(struct pimax_ctrl_ctx *ctx, const pimax_controller_status *s)
{
	if (ctx == NULL || (s->slot != PIMAX_SLOT_LEFT && s->slot != PIMAX_SLOT_RIGHT)) {
		return;
	}
	struct pimax_controller *c = ctx->ctl[s->slot == PIMAX_SLOT_LEFT ? 0 : 1];
	if (c != NULL) {
		atomic_store(&c->last_status_ns, os_monotonic_get_ns());
	}
}

void
pimax_ctrl_on_state(struct pimax_ctrl_ctx *ctx, pimax_state state)
{
	if (ctx != NULL && state == PIMAX_CONNECTED) {
		atomic_store(&ctx->need_attach_all, true);
	}
}

/*
 * Attach thread: libpimax_controller_attach() sleeps, so it cannot run on the session thread.
 */

static void
do_attach(struct pimax_ctrl_ctx *ctx, struct pimax_controller *c, const char *why)
{
	if (c == NULL) {
		return;
	}
	atomic_store(&c->last_attach_ns, os_monotonic_get_ns());
	int rc = pimax_controller_attach(ctx->lib, c->slot);
	unsigned n = atomic_fetch_add(&c->attaches, 1) + 1;
	U_LOG_IFL_I(ctx->log_level, "controller slot %u attach burst #%u (%s) -> %d", c->slot, n, why, rc);
}

static void *
attach_thread(void *arg)
{
	struct pimax_ctrl_ctx *ctx = arg;
	atomic_store(&ctx->need_attach_all, true);

	while (!atomic_load(&ctx->stop)) {
		uint64_t now = os_monotonic_get_ns();
		if (atomic_exchange(&ctx->need_attach_all, false)) {
			for (int i = 0; i < 2 && !atomic_load(&ctx->stop); i++) {
				do_attach(ctx, ctx->ctl[i], "start/reconnect");
			}
		}
		for (int i = 0; i < 2 && !atomic_load(&ctx->stop); i++) {
			struct pimax_controller *c = ctx->ctl[i];
			if (c == NULL) {
				continue;
			}
			pthread_mutex_lock(&c->lock);
			uint64_t last_rep = c->have_report ? c->last_report_ns : 0;
			pthread_mutex_unlock(&c->lock);
			uint64_t st = atomic_load(&c->last_status_ns);
			uint64_t at = atomic_load(&c->last_attach_ns);
			// 0x37 heartbeat = awake but not attached. Valid if newer than our last attach and than the
			// last report, with no report for >1.5 s.
			if (st > at && st > last_rep && now - last_rep > 1500ull * 1000 * 1000 && now - at > REATTACH_MIN_NS) {
				do_attach(ctx, c, "0x37 heartbeat without reports");
			}
		}
		struct timespec ts = {0, 100 * 1000 * 1000};
		nanosleep(&ts, NULL);
	}
	return NULL;
}

void
pimax_ctrl_start(struct pimax_ctrl_ctx *ctx, pimax_t *lib)
{
	ctx->lib = lib;
	atomic_store(&ctx->stop, false);
	if (pthread_create(&ctx->thread, NULL, attach_thread, ctx) == 0) {
		ctx->thread_running = true;
	} else {
		U_LOG_IFL_E(ctx->log_level, "pimax controllers: could not start attach thread");
	}
}

void
pimax_ctrl_stop(struct pimax_ctrl_ctx *ctx)
{
	if (ctx == NULL) {
		return;
	}
	if (ctx->thread_running) {
		atomic_store(&ctx->stop, true);
		pthread_join(ctx->thread, NULL);
		ctx->thread_running = false;
	}
	ctx->lib = NULL;
	ctx->head = NULL; // the HMD is going away
	ctx_unref(ctx);
}

/*
 * Creation.
 */

static struct pimax_controller *
ctl_create(struct pimax_ctrl_ctx *ctx, int index)
{
	bool left = index == 0;
	// The hand tracking input is the last one; leave it out when hand tracking is off.
	const int input_count = ctx->hands ? I_COUNT : I_HAND_TRACKING;
	struct pimax_controller *c =
	    U_DEVICE_ALLOCATE(struct pimax_controller, U_DEVICE_ALLOC_TRACKING_NONE, input_count, 1);
	if (c == NULL) {
		return NULL;
	}
	c->ctx = ctx;
	c->index = index;
	c->slot = left ? PIMAX_SLOT_LEFT : PIMAX_SLOT_RIGHT;
	pthread_mutex_init(&c->lock, NULL);
	c->imu_rot = (struct xrt_quat)XRT_QUAT_IDENTITY;
	c->yaw_ref = (struct xrt_quat)XRT_QUAT_IDENTITY;

	struct xrt_input *in = c->base.inputs;
	in[I_BTN1_CLICK].name = left ? XRT_INPUT_TOUCH_X_CLICK : XRT_INPUT_TOUCH_A_CLICK;
	in[I_BTN1_TOUCH].name = left ? XRT_INPUT_TOUCH_X_TOUCH : XRT_INPUT_TOUCH_A_TOUCH;
	in[I_BTN2_CLICK].name = left ? XRT_INPUT_TOUCH_Y_CLICK : XRT_INPUT_TOUCH_B_CLICK;
	in[I_BTN2_TOUCH].name = left ? XRT_INPUT_TOUCH_Y_TOUCH : XRT_INPUT_TOUCH_B_TOUCH;
	in[I_MENU].name = left ? XRT_INPUT_TOUCH_MENU_CLICK : XRT_INPUT_TOUCH_SYSTEM_CLICK;
	in[I_SQUEEZE].name = XRT_INPUT_TOUCH_SQUEEZE_VALUE;
	in[I_TRIGGER_TOUCH].name = XRT_INPUT_TOUCH_TRIGGER_TOUCH;
	in[I_TRIGGER_VALUE].name = XRT_INPUT_TOUCH_TRIGGER_VALUE;
	in[I_STICK_CLICK].name = XRT_INPUT_TOUCH_THUMBSTICK_CLICK;
	in[I_STICK_TOUCH].name = XRT_INPUT_TOUCH_THUMBSTICK_TOUCH;
	in[I_STICK].name = XRT_INPUT_TOUCH_THUMBSTICK;
	in[I_GRIP_POSE].name = XRT_INPUT_TOUCH_GRIP_POSE;
	in[I_AIM_POSE].name = XRT_INPUT_TOUCH_AIM_POSE;
	for (int i = 0; i < input_count; i++) {
		in[i].active = (i >= I_GRIP_POSE);
	}
	c->base.outputs[0].name = XRT_OUTPUT_NAME_TOUCH_HAPTIC;

	// Share the HMD's tracking origin so the controllers sit in the head frame (poses are head-relative); otherwise
	// the builder adds its default 3DoF hand offsets (+-0.2, 1.3, -0.5) on top of ours. The origin lives inside the
	// HMD allocation, which outlives every use (it is only read while the device list is alive).
	if (ctx->head != NULL && ctx->head->tracking_origin != NULL) {
		c->base.tracking_origin = ctx->head->tracking_origin;
	}

	c->base.name = XRT_DEVICE_TOUCH_CONTROLLER;
	c->base.device_type = left ? XRT_DEVICE_TYPE_LEFT_HAND_CONTROLLER : XRT_DEVICE_TYPE_RIGHT_HAND_CONTROLLER;
	snprintf(c->base.str, XRT_DEVICE_NAME_LEN, "Pimax Dream Air %s Controller", left ? "Left" : "Right");
	snprintf(c->base.serial, XRT_DEVICE_NAME_LEN, "pimax-ctrl-%s", left ? "left" : "right");
	c->base.binding_profiles = left ? profiles_left : profiles_right;
	c->base.binding_profile_count = 2;
	c->base.supported.orientation_tracking = ctx->imu_mode;
	c->base.supported.position_tracking = false;
	c->base.supported.battery_status = true;

	if (ctx->hands) {
		c->hands = pimax_hands_create();
		if (c->hands == NULL) {
			U_LOG_IFL_E(ctx->log_level, "pimax-hands: out of memory, hand tracking off for this controller");
			c->base.input_count = I_HAND_TRACKING; // drop the unused hand tracking input
		}
	}
	if (c->hands != NULL) {
		pimax_hands_get(c->hands, &c->hands_out);
		pimax_hands_pose p;
		pimax_hands_grip_to_wrist(left ? 0 : 1, &p);
		c->grip_to_wrist.orientation = (struct xrt_quat){p.orientation[0], p.orientation[1], p.orientation[2],
		                                                  p.orientation[3]};
		c->grip_to_wrist.position = (struct xrt_vec3){p.position[0], p.position[1], p.position[2]};
		in[I_HAND_TRACKING].name = left ? XRT_INPUT_HT_CONFORMING_LEFT : XRT_INPUT_HT_CONFORMING_RIGHT;
		in[I_HAND_TRACKING].active = true;
		c->base.get_hand_tracking = pimax_ctl_get_hand_tracking;
		c->base.supported.hand_tracking = true;
	}

	c->base.update_inputs = pimax_ctl_update_inputs;
	c->base.get_tracked_pose = pimax_ctl_get_tracked_pose;
	c->base.set_output = pimax_ctl_set_output;
	c->base.get_battery_status = pimax_ctl_get_battery_status;
	c->base.destroy = pimax_ctl_destroy;

	u_var_add_root(c, c->base.str, true);
	u_var_add_f32(c, &c->trigger, "trigger");
	u_var_add_f32(c, &c->grip, "grip");
	u_var_add_f32(c, &c->stick_x, "stick x");
	u_var_add_f32(c, &c->stick_y, "stick y");
	u_var_add_bool(c, &c->btn_a, "A/X");
	u_var_add_bool(c, &c->btn_b, "B/Y");
	u_var_add_bool(c, &c->btn_menu, "menu");
	if (c->hands != NULL) {
		u_var_add_ro_f32(c, &c->hands_out.curl[3], "hand: index curl");
		u_var_add_ro_f32(c, &c->hands_out.curl[2], "hand: middle curl");
		u_var_add_ro_f32(c, &c->hands_out.curl[4], "hand: thumb curl");
	}
	return c;
}

bool
pimax_ctrl_demo_enabled(void)
{
	return debug_get_bool_option_pimax_ctrl_hands() && debug_get_bool_option_pimax_ctrl_hands_demo();
}

struct pimax_ctrl_ctx *
pimax_ctrl_create(struct xrt_device *head, enum u_logging_level log_level, struct xrt_device *out_devs[2])
{
	struct pimax_ctrl_ctx *ctx = U_TYPED_CALLOC(struct pimax_ctrl_ctx);
	if (ctx == NULL) {
		return NULL;
	}
	ctx->ref.count = 3; // HMD + 2 controllers
	ctx->head = head;
	ctx->log_level = log_level;
	const char *mode = debug_get_option_pimax_ctrl_pose();
	ctx->imu_mode = mode != NULL && strcmp(mode, "imu") == 0;
	float lsb = debug_get_float_option_pimax_ctrl_gyro_lsb_dps();
	ctx->gyro_scale = (float)(M_PI / 180.0) / (lsb > 0.0f ? lsb : 16.4f);
	ctx->hands = debug_get_bool_option_pimax_ctrl_hands();
	ctx->hands_demo = ctx->hands && debug_get_bool_option_pimax_ctrl_hands_demo();
	const char *feed = debug_get_option_pimax_ctrl_hands_feed();
	ctx->hands_eager = ctx->hands && feed != NULL && strcmp(feed, "report") == 0;
	if (ctx->hands && pimax_hands_abi_version() != PIMAX_HANDS_ABI_VERSION) {
		U_LOG_IFL_E(log_level, "pimax-hands ABI %u != header %u: stale libpimax_capi.a, hand tracking off",
		            pimax_hands_abi_version(), PIMAX_HANDS_ABI_VERSION);
		ctx->hands = ctx->hands_demo = false;
	}

	for (int i = 0; i < 2; i++) {
		ctx->ctl[i] = ctl_create(ctx, i);
		if (ctx->ctl[i] == NULL) {
			// Not worth unwinding in detail; this only fails on OOM.
			return NULL;
		}
		out_devs[i] = &ctx->ctl[i]->base;
	}
	U_LOG_IFL_I(log_level, "Pimax controllers: pose mode %s%s, hand tracking %s", ctx->imu_mode ? "imu" : "fixed",
	            ctx->imu_mode ? " (gyro scale is a guess)" : "",
	            ctx->hands ? (ctx->hands_demo ? "on (DEMO pattern)"
	                                          : (ctx->hands_eager ? "on (data source: controller, feed per report)"
	                                                              : "on (data source: controller, lazy feed)"))
	                       : "off");
	return ctx;
}
