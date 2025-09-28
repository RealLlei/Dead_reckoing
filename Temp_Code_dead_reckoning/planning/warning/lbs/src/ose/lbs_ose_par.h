#pragma once
#ifndef LBS_OSE_PAE_H
#define LBS_OSE_PAE_H

// #define OSE_NUM_OBJECTS (1u) // OSE_NUM_OBJECTS --> OSE_LBS_NUM_OBJECTS 80
#define OSE_EM_GEN_OBJECT_MT_STATE_DELETED (0u)
#define OSE_NUM_OF_BREAK_LINES (2u)
#define OSE_NUM_OF_WARN_LEVELS (3u)
#define OSE_DOOR_LOCKS_NUMBER (4u)

#define OSE_PC_IDX (0u)   // OBJ is pedestrian and cyclist
#define OSE_VEH_IDX (1u)  // OBJ is CAR
#define OSE_NOESTIM_IDX (2u)

#define OSE_MAX_UPDATE_COUNTER (60000u)
#define OSE_MIN_HEADING_PRED (0.0872f)

// Side track classification defines
#define OSE_FRONTOBJ_ACTIVETION_THRESH (0.9f)
#define OSE_FRONTOBJ_DEACTIVETION_THRESH (0.7f)
#define OSE_FRONTOBJ_INIT_PROB (0.3f)
// Thresholds for the approach angle
#define OSE_APPROACH_ANGLE_MARGIN (5.0f)
#define OSE_APPROACH_ANGLE_HYST (10.0f)
// Mirror check defines
#define OSE_MIRROR_ACTIVATION_THRESH (0.8f)
#define OSE_MIRROR_DEACTIVATION_THRESH (0.5f)
// For object which have a warning the crossing line is enlarged using this
// hysteresis
#define OSE_BREAKTHROUGH_HYSTERESIS (0.3f)
// Edge of the field of
#define OSE_CONFI_UPDATE_MAX_EFOV (20.0f)
#define OSE_CONFI_UPDATE_MIN_EFOV (10.0f)  // 2 -->10
// Confidence check definess
#define OSE_CONFI_UPDATE_MAX (30.0f)  // 20 --> 15
#define OSE_CONFI_UPDATE_MIN (25.0f)  // 5 --> 15
// For objects below this threshold the warning is not interrupted
#define OSE_MULTIOBJ_SAFE_MARGIN (0.7f)
// Side track classification defines
#define OSE_FRONTOBJ_IMPLAUSIBLE_THRESH (0.05f)
#define OSE_FRONTOBJ_INC_PROB (0.1f)
#define OSE_FRONTOBJ_DEC_PROB (0.02f)
// Edge of the field of view defines
#define OSE_TTC_EFOV_THRESH (1.0f)
// Objects which fulfill all warning condition shall not switch on a warning if
// the TTC is smaller than this threshold
#define OSE_SHORT_WARNING_THRESH (0.0f)
// Confidence check defines
// Threshold for set flag for confidence hit
#define OSE_HIT_CONFI_THRESH (70u)
// Min RCS value of an object to be considered relevant
#define OSE_RCS_THRESH (-20.0f)
// Define how many cycles a warning shall be suppressed for multiple following
// targets 0 means no interruption
#define OSE_MULTIOBJ_INTERRUPT_CYCLES (1u)
#endif