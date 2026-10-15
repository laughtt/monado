// Copyright 2026, laughtt
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Room anchor hook (pimax-re rust/pimax-anchor): world offset for the head pose, plus a ring of the raw head
 *         poses so the tool can pair a camera frame with the head pose at that instant. Header only, included by
 *         pimax_hmd.c. Layout of both files: rust/pimax-anchor/src/world.rs (keep in sync).
 *
 * PIMAX_ANCHOR=1 enables it. Then:
 *  - at start the offset file ($PIMAX_ANCHOR_FILE or $XDG_RUNTIME_DIR/pimax-anchor.offset) is deleted (the SLAM world
 *    origin is new every run, an old offset would be wrong) and the head ring ($PIMAX_ANCHOR_HEAD_FILE or
 *    $XDG_RUNTIME_DIR/pimax-anchor.head) is created: 16 byte header (u32 magic 0x434E4150, u32 version 1, u32 seq (odd
 *    while writing), u32 pid), 128 slots of 40 bytes (u64 ts_ns, f32 pos[3], f32 quat[4] xyzw, u32 flags bit0 =
 *    position valid), round robin, raw (before the offset) poses, at most one every 3 ms;
 *  - `pimax-anchor apply` writes the offset file ("pimax-anchor-offset 1", "pid N", "pose px py pz qx qy qz qw");
 *    it is re-read when it changes (checked every 250 ms) and applied as app_from_slam = offset * slam to the head
 *    pose, so controllers, hands-in-head-frame and the gaze ray move with it. Ignored unless pid is this process.
 *  - small changes of the offset glide instead of jumping (the floor lock and periodic re-fixes write small steps):
 *    the applied offset moves toward the file's at most PIMAX_ANCHOR_SLEW m/s (default 0.05) and
 *    PIMAX_ANCHOR_SLEW_DEG deg/s (default 2), i.e. under 1 mm / 0.03 deg per 90 Hz frame. A change larger than
 *    PIMAX_ANCHOR_SNAP m (default 0.25) or 5 deg, and the first offset of a session, is applied at once (a deliberate
 *    placement). PIMAX_ANCHOR_SLEW=0 applies every change at once (the old behaviour).
 * @ingroup drv_pimax
 */

#pragma once

#include <fcntl.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include "math/m_api.h"
#include "math/m_vec3.h"

#include <math.h>
#include "os/os_time.h"
#include "util/u_logging.h"
#include "xrt/xrt_defines.h"

#define PIMAX_ANCHOR_MAGIC 0x434E4150u
#define PIMAX_ANCHOR_SLOTS 128
#define PIMAX_ANCHOR_HDR 16
#define PIMAX_ANCHOR_SLOT 40
#define PIMAX_ANCHOR_SIZE (PIMAX_ANCHOR_HDR + PIMAX_ANCHOR_SLOTS * PIMAX_ANCHOR_SLOT)

struct pimax_anchor_state
{
	pthread_mutex_t lock;
	bool inited;
	bool enabled;
	uint8_t *ring;
	uint32_t next;
	int64_t last_ring_ns;
	char offset_path[512];
	int64_t last_check_ns;
	time_t mtime_s;
	long mtime_ns;
	bool have_offset;
	struct xrt_pose offset;
	// what is applied now (moves toward `offset`)
	bool have_applied;
	struct xrt_pose applied;
	int64_t last_step_ns;
	float slew_m_s;
	float slew_deg_s;
	float snap_m;
};

static struct pimax_anchor_state g_pimax_anchor = {.lock = PTHREAD_MUTEX_INITIALIZER};

static inline void
pimax_anchor_path(const char *envname, const char *leaf, char *out, size_t n)
{
	const char *e = getenv(envname);
	if (e != NULL && e[0] != '\0') {
		snprintf(out, n, "%s", e);
		return;
	}
	const char *d = getenv("XDG_RUNTIME_DIR");
	snprintf(out, n, "%s/%s", (d != NULL && d[0] != '\0') ? d : "/tmp", leaf);
}

//! Call with the lock held.
static inline void
pimax_anchor_init_locked(struct pimax_anchor_state *s)
{
	s->inited = true;
	const char *on = getenv("PIMAX_ANCHOR");
	if (on == NULL || strcmp(on, "1") != 0) {
		return;
	}
	pimax_anchor_path("PIMAX_ANCHOR_FILE", "pimax-anchor.offset", s->offset_path, sizeof(s->offset_path));
	unlink(s->offset_path);
	char ring_path[512];
	pimax_anchor_path("PIMAX_ANCHOR_HEAD_FILE", "pimax-anchor.head", ring_path, sizeof(ring_path));
	unlink(ring_path);
	int fd = open(ring_path, O_RDWR | O_CREAT | O_TRUNC | O_CLOEXEC, 0600);
	if (fd < 0 || ftruncate(fd, PIMAX_ANCHOR_SIZE) != 0) {
		U_LOG_E("PIMAX_ANCHOR: cannot create %s", ring_path);
		if (fd >= 0) {
			close(fd);
		}
		return;
	}
	void *p = mmap(NULL, PIMAX_ANCHOR_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	close(fd);
	if (p == MAP_FAILED) {
		U_LOG_E("PIMAX_ANCHOR: mmap failed");
		return;
	}
	s->ring = (uint8_t *)p;
	const char *e = getenv("PIMAX_ANCHOR_SLEW");
	s->slew_m_s = (e != NULL && e[0] != '\0') ? (float)atof(e) : 0.05f;
	e = getenv("PIMAX_ANCHOR_SLEW_DEG");
	s->slew_deg_s = (e != NULL && e[0] != '\0') ? (float)atof(e) : 2.0f;
	e = getenv("PIMAX_ANCHOR_SNAP");
	s->snap_m = (e != NULL && e[0] != '\0') ? (float)atof(e) : 0.25f;
	uint32_t hdr[4] = {PIMAX_ANCHOR_MAGIC, 1, 0, (uint32_t)getpid()};
	memcpy(s->ring, hdr, sizeof(hdr));
	s->enabled = true;
	U_LOG_I("PIMAX_ANCHOR=1: head ring %s, offset file %s", ring_path, s->offset_path);
}

static inline void
pimax_anchor_refresh_offset_locked(struct pimax_anchor_state *s, int64_t now_ns)
{
	if (now_ns - s->last_check_ns < 250 * 1000 * 1000) {
		return;
	}
	s->last_check_ns = now_ns;
	struct stat st;
	if (stat(s->offset_path, &st) != 0) {
		s->have_offset = false;
		s->have_applied = false;
		s->mtime_s = 0;
		return;
	}
	if (s->have_offset && st.st_mtim.tv_sec == s->mtime_s && st.st_mtim.tv_nsec == s->mtime_ns) {
		return;
	}
	s->mtime_s = st.st_mtim.tv_sec;
	s->mtime_ns = st.st_mtim.tv_nsec;
	FILE *f = fopen(s->offset_path, "r");
	if (f == NULL) {
		return;
	}
	char line[128];
	unsigned pid = 0;
	float v[7];
	bool ok = fgets(line, sizeof(line), f) != NULL && strncmp(line, "pimax-anchor-offset 1", 21) == 0 &&
	          fgets(line, sizeof(line), f) != NULL && sscanf(line, "pid %u", &pid) == 1 &&
	          fgets(line, sizeof(line), f) != NULL &&
	          sscanf(line, "pose %f %f %f %f %f %f %f", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6]) == 7;
	fclose(f);
	if (!ok || pid != (unsigned)getpid()) {
		s->have_offset = false;
		s->have_applied = false;
		return;
	}
	s->offset.position = (struct xrt_vec3){v[0], v[1], v[2]};
	s->offset.orientation = (struct xrt_quat){v[3], v[4], v[5], v[6]};
	math_quat_normalize(&s->offset.orientation);
	s->have_offset = true;
	U_LOG_I("PIMAX_ANCHOR: offset (%.3f %.3f %.3f) quat (%.3f %.3f %.3f %.3f)", v[0], v[1], v[2], v[3], v[4], v[5],
	        v[6]);
}

static inline void
pimax_anchor_record_locked(struct pimax_anchor_state *s, int64_t at_ns, const struct xrt_space_relation *rel)
{
	if (at_ns <= 0 || (rel->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) == 0) {
		return;
	}
	int64_t d = at_ns - s->last_ring_ns;
	if (d < 3 * 1000 * 1000 && d > -3 * 1000 * 1000) {
		return;
	}
	s->last_ring_ns = at_ns;
	uint8_t *slot = s->ring + PIMAX_ANCHOR_HDR + (size_t)(s->next % PIMAX_ANCHOR_SLOTS) * PIMAX_ANCHOR_SLOT;
	s->next++;
	_Atomic uint32_t *seq = (_Atomic uint32_t *)(void *)(s->ring + 8);
	uint32_t q0 = atomic_load_explicit(seq, memory_order_relaxed);
	atomic_store_explicit(seq, q0 + 1, memory_order_release);
	uint64_t ts = (uint64_t)at_ns;
	float f[7] = {rel->pose.position.x,    rel->pose.position.y,     rel->pose.position.z,   rel->pose.orientation.x,
	              rel->pose.orientation.y, rel->pose.orientation.z, rel->pose.orientation.w};
	uint32_t flags = (rel->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) ? 1u : 0u;
	memcpy(slot, &ts, 8);
	memcpy(slot + 8, f, 28);
	memcpy(slot + 36, &flags, 4);
	atomic_store_explicit(seq, q0 + 2, memory_order_release);
}

//! Angle between two unit quaternions (rad).
static inline float
pimax_anchor_quat_angle(const struct xrt_quat *a, const struct xrt_quat *b)
{
	float d = fabsf(a->x * b->x + a->y * b->y + a->z * b->z + a->w * b->w);
	return 2.0f * acosf(d > 1.0f ? 1.0f : d);
}

/*!
 * Move the applied offset toward the file's (call with the lock held): glide small changes, snap big ones.
 */
static inline void
pimax_anchor_step_locked(struct pimax_anchor_state *s, int64_t now_ns)
{
	float dt = s->last_step_ns > 0 ? (float)(now_ns - s->last_step_ns) * 1e-9f : 0.0f;
	s->last_step_ns = now_ns;
	if (dt < 0.0f) {
		dt = 0.0f;
	}
	if (dt > 0.1f) {
		dt = 0.1f;
	}
	struct xrt_pose *a = &s->applied;
	const struct xrt_pose *t = &s->offset;
	struct xrt_vec3 dp = m_vec3_sub(t->position, a->position);
	float dist = m_vec3_len(dp);
	float ang = pimax_anchor_quat_angle(&a->orientation, &t->orientation);
	if (!s->have_applied || s->slew_m_s <= 0.0f || dist > s->snap_m || ang > 5.0f * (float)M_PI / 180.0f) {
		if (s->have_applied && (dist > 0.002f || ang > 0.001f)) {
			U_LOG_I("PIMAX_ANCHOR: offset placed at once (%.3f m, %.2f deg)", dist, ang * 180.0f / (float)M_PI);
		}
		*a = *t;
		s->have_applied = true;
		return;
	}
	float max_d = s->slew_m_s * dt;
	if (dist <= max_d) {
		a->position = t->position;
	} else {
		a->position = m_vec3_add(a->position, m_vec3_mul_scalar(dp, max_d / dist));
	}
	float max_a = s->slew_deg_s * (float)M_PI / 180.0f * dt;
	if (ang <= max_a || ang < 1e-6f) {
		a->orientation = t->orientation;
	} else {
		struct xrt_quat q;
		math_quat_slerp(&a->orientation, &t->orientation, max_a / ang, &q);
		math_quat_normalize(&q);
		a->orientation = q;
	}
}

/*!
 * Record the raw relation (SLAM world) and apply the offset to it in place. Cheap when PIMAX_ANCHOR is not set.
 */
static inline void
pimax_anchor_process(int64_t at_ns, struct xrt_space_relation *rel)
{
	struct pimax_anchor_state *s = &g_pimax_anchor;
	pthread_mutex_lock(&s->lock);
	if (!s->inited) {
		pimax_anchor_init_locked(s);
	}
	if (!s->enabled) {
		pthread_mutex_unlock(&s->lock);
		return;
	}
	pimax_anchor_record_locked(s, at_ns, rel);
	int64_t now_ns = (int64_t)os_monotonic_get_ns();
	pimax_anchor_refresh_offset_locked(s, now_ns);
	bool have = s->have_offset;
	if (have) {
		pimax_anchor_step_locked(s, now_ns);
	}
	struct xrt_pose off = s->applied;
	pthread_mutex_unlock(&s->lock);
	if (!have) {
		return;
	}
	if (rel->relation_flags & XRT_SPACE_RELATION_POSITION_VALID_BIT) {
		struct xrt_pose out;
		math_pose_transform(&off, &rel->pose, &out);
		rel->pose = out;
	} else if (rel->relation_flags & XRT_SPACE_RELATION_ORIENTATION_VALID_BIT) {
		struct xrt_quat q;
		math_quat_rotate(&off.orientation, &rel->pose.orientation, &q);
		rel->pose.orientation = q;
	}
	if (rel->relation_flags & XRT_SPACE_RELATION_LINEAR_VELOCITY_VALID_BIT) {
		struct xrt_vec3 v;
		math_quat_rotate_vec3(&off.orientation, &rel->linear_velocity, &v);
		rel->linear_velocity = v;
	}
	if (rel->relation_flags & XRT_SPACE_RELATION_ANGULAR_VELOCITY_VALID_BIT) {
		struct xrt_vec3 v;
		math_quat_rotate_vec3(&off.orientation, &rel->angular_velocity, &v);
		rel->angular_velocity = v;
	}
}
