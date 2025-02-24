/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file dataman.h
 *
 * DATAMANAGER driver.
 */
#pragma once

#include <string.h>
#include <navigator/navigation.h>
#include <uORB/topics/mission.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Types of items that the data manager can store */
typedef enum {
	DM_KEY_SAFE_POINTS = 0,		/* Safe points coordinates, safe point 0 is home point */
	DM_KEY_FENCE_POINTS,		/* Fence vertex coordinates */
	DM_KEY_WAYPOINTS_OFFBOARD_0,	/* Mission way point coordinates sent over mavlink */
	DM_KEY_WAYPOINTS_OFFBOARD_1,	/* (alternate between 0 and 1) */
	DM_KEY_MISSION_STATE,		/* Persistent mission state */
	DM_KEY_COMPAT,
	DM_KEY_NUM_KEYS			/* Total number of item types defined */
} dm_item_t;

#if defined(MEMORY_CONSTRAINED_SYSTEM)
enum {
	DM_KEY_SAFE_POINTS_MAX = 8,
	DM_KEY_FENCE_POINTS_MAX = 16,
	DM_KEY_WAYPOINTS_OFFBOARD_0_MAX = NUM_MISSIONS_SUPPORTED,
	DM_KEY_WAYPOINTS_OFFBOARD_1_MAX = NUM_MISSIONS_SUPPORTED,
	DM_KEY_MISSION_STATE_MAX = 1,
	DM_KEY_COMPAT_MAX = 1
};
#else
/** The maximum number of instances for each item type */
enum {
	DM_KEY_SAFE_POINTS_MAX = 512,
	DM_KEY_FENCE_POINTS_MAX = 512,
	DM_KEY_WAYPOINTS_OFFBOARD_0_MAX = NUM_MISSIONS_SUPPORTED,
	DM_KEY_WAYPOINTS_OFFBOARD_1_MAX = NUM_MISSIONS_SUPPORTED,
	DM_KEY_MISSION_STATE_MAX = 1,
	DM_KEY_COMPAT_MAX = 1
};
#endif

struct dataman_compat_s {
	uint64_t key;
};

/* increment this define whenever a binary incompatible change is performed */
#define DM_COMPAT_VERSION	3ULL

#define DM_COMPAT_KEY_2ULL 0x238106188 // old version
#define DM_COMPAT_KEY ((DM_COMPAT_VERSION << 32) + (sizeof(struct mission_item_s) << 24) + \
		       (sizeof(struct mission_s) << 16) + (sizeof(struct mission_stats_entry_s) << 12) + \
		       (sizeof(struct mission_fence_point_s) << 8) + (sizeof(struct mission_safe_point_s) << 4) + \
		       sizeof(struct dataman_compat_s))

/** Retrieve from the data manager store */
__EXPORT ssize_t
dm_read(
	dm_item_t item,			/* The item type to retrieve */
	unsigned index,			/* The index of the item */
	void *buffer,			/* Pointer to caller data buffer */
	size_t buflen			/* Length in bytes of data to retrieve */
);

/** write to the data manager store */
__EXPORT ssize_t
dm_write(
	dm_item_t  item,		/* The item type to store */
	unsigned index,			/* The index of the item */
	const void *buffer,		/* Pointer to caller data buffer */
	size_t buflen			/* Length in bytes of data to retrieve */
);

/**
 * Lock all items of a type. Can be used for atomic updates of multiple items (single items are always updated
 * atomically).
 * Note that this lock is independent from dm_read & dm_write calls.
 * @return 0 on success and lock taken, -1 on error (lock not taken, errno set)
 */
__EXPORT int
dm_lock(
	dm_item_t item			/* The item type to lock */
);

/**
 * Try to lock all items of a type (@see sem_trywait()).
 * @return 0 if lock is taken, -1 otherwise (on error or if already locked. errno is set accordingly)
 */
__EXPORT int
dm_trylock(
	dm_item_t item			/* The item type to lock */
);

/** Unlock all items of a type */
__EXPORT void
dm_unlock(
	dm_item_t item			/* The item type to unlock */
);

/** Erase all items of this type */
__EXPORT int
dm_clear(
	dm_item_t item			/* The item type to clear */
);

#ifdef __cplusplus
}
#endif
