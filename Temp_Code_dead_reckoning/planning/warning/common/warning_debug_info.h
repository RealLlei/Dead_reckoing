/******************************************************************************
 * Copyright 2022 The TL Authors. All Rights Reserved.
 *****************************************************************************/
#include "planning/localview/local_view.h"
#include "planning/warning/cta/cta.h"
#include "planning/warning/lbs/common/tue_common_libs.h"
#include "planning/warning/lbs/src/bsd/lbs_bsd_main.h"
#include "planning/warning/lbs/src/bsd/lbs_bsd_par.h"
#include "planning/warning/lbs/src/lbs_calculation.h"
#include "planning/warning/lbs/src/lbs_main.h"
#include "planning/warning/lbs/src/lbs_par.h"
#include "planning/warning/lbs/src/lbs_wrapper.h"
#include "planning/warning/lbs/src/rcw/lbs_rcw_calculation.h"
#include "planning/warning/lbs/src/rcw/lbs_rcw_main.h"

extern LBSInReq_st reqPorts;
extern LBSParam_st params;
extern LBSOutPro_t proPorts;
extern LBSDebug_t debugInfo;
extern CTA_Switch CTA_function_switch;
extern LBS_Switch LBS_function_switch;

// CTA interface
extern CTAInReq_t CTA_reqPorts;
extern CTAParam_t CTA_params;
extern CTAOutPro_t CTA_proPorts;
extern CTADebug_t CTA_debugInfo;
extern FCTAInReq_t FCTAreqPorts;
extern RCTAInReq_t RCTAreqPorts;

namespace TL {
namespace planning {
namespace warning {
namespace WarningDebugInfo {
/**
 * @brief determine if the warning function is active
 *
 * @param localview the input of planning
 * @param warning_fault warning system fault
 * @param warning_output the output result of warning
 * @return true the warning function is active
 * @return false the warning function is not active
 */
bool WarningStatus(const std::shared_ptr<LocalView>& localview,
                   const soc::WarningFault& warning_fault,
                   WarningOutput* const warning_output);
/**
 * @brief BSD debug information output to cyber monitor
 *
 * @param localview the input of planning
 * @param warning_output the output result of warning
 * @return true
 */
bool BSDDebugInfo(const std::shared_ptr<LocalView>& localview,
                  WarningOutput* const warning_output);
/**
 * @brief RCW debug information output to cyber monitor
 *
 * @param localview the input of planning
 * @param warning_output the output result of warning
 * @return true
 */
bool RCWDebugInfo(const std::shared_ptr<LocalView>& localview,
                  WarningOutput* const warning_output);
/**
 * @brief DOW debug information output to cyber monitor
 *
 * @param localview the input of planning
 * @param warning_output the output result of warning
 * @return true
 */
bool DOWDebugInfo(int perception_obstacles_size, WarningOutput* warning_output);
/**
 * @brief FCTA debug information output to cyber monitor
 *
 * @param localview the input of planning
 * @param warning_output the output result of warning
 * @return true
 */
bool FCTADebugInfo(int perception_obstacles_size,
                   WarningOutput* warning_output);
/**
 * @brief LCA debug information output to cyber monitor
 *
 * @param localview the input of planning
 * @param warning_output the output result of warning
 * @return true
 */
bool LCADebugInfo(const std::shared_ptr<LocalView>& localview,
                  WarningOutput* const warning_output);
/**
 * @brief RCTA debug information output to cyber monitor
 *
 * @param localview the input of planning
 * @param warning_output the output result of warning
 * @return true
 */
bool RCTADebugInfo(const std::shared_ptr<LocalView>& localview,
                   WarningOutput* const warning_output);
}  // namespace WarningDebugInfo
}  // namespace warning
}  // namespace planning
}  // namespace TL
