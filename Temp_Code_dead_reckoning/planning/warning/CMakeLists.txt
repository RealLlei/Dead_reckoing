cmake_minimum_required(VERSION 3.12.1)
project(europa_warning)
set(CMAKE_CXX_FLAGS "-w ${CMAKE_CXX_FLAGS}")
message("Warning Compiler Options: " ${CMAKE_CXX_FLAGS})

add_library(
  ${PROJECT_NAME} 
  SHARED 
  warning.cc
  common/define.c
  common/warning_debug_info.cc
  common/warning_util.cc
  lbs/src/si/lbs_si_calculation.c
  lbs/src/si/lbs_si_calculation.h
  lbs/src/si/lbs_si_ext.h
  lbs/src/si/lbs_si_main.c
  lbs/src/si/lbs_si_main.h
  lbs/src/si/lbs_si_par.h
  lbs/src/si/lbs_si.h
  lbs/src/lca/lbs_lca_calculation.c
  lbs/src/lca/lbs_lca_calculation.h
  lbs/src/lca/lbs_lca_ext.h
  lbs/src/lca/lbs_lca_main.c
  lbs/src/lca/lbs_lca_main.h
  lbs/src/lca/lbs_lca_par.h

  lbs/src/bsd/lbs_bsd.h
  lbs/src/bsd/lbs_bsd_calculation.c
  lbs/src/bsd/lbs_bsd_calculation.h
  lbs/src/bsd/lbs_bsd_ext.h
  lbs/src/bsd/lbs_bsd_main.c
  lbs/src/bsd/lbs_bsd_main.h
  lbs/src/bsd/lbs_bsd_par.h
  lbs/src/bsd/lbs_bsd.h
  lbs/src/ose/lbs_ose.h
  lbs/src/ose/lbs_ose_calculation.c
  lbs/src/ose/lbs_ose_calculation.h
  lbs/src/ose/lbs_ose_ext.h
  lbs/src/ose/lbs_ose_par.h
  lbs/src/ose/lbs_ose.c
  lbs/src/rcw/lbs_rcw_calculation.h
  lbs/src/rcw/lbs_rcw_calculation.c
  lbs/src/rcw/lbs_rcw_ext.h
  lbs/src/rcw/lbs_rcw_main.h
  lbs/src/rcw/lbs_rcw_main.c
  lbs/src/lbs_calculation.c
  lbs/src/lbs_calculation.h
  lbs/src/lbs_external.h
  lbs/src/lbs_main.c
  lbs/src/lbs_main.h
  lbs/src/lbs_par.h
  lbs/src/lbs_wrapper.c
  lbs/src/lbs_wrapper.h
  lbs/common/tue_common_libs.c
  lbs/common/tue_common_libs.h
  lbs/common/TM_Global_Types.h
  cta/cta.c
  cta/cta.h
  cta/cta_par.h
  cta/cta_extern.h
  cta/fcta/cta_fcta_extern.h
  cta/fcta/cta_fcta.c
  cta/fcta/cta_fcta.h
  cta/rcta/cta_rcta_extern.h
  cta/rcta/cta_rcta.c
  cta/rcta/cta_rcta.h

)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(
  ${PROJECT_NAME}
  globalproto
  ${GLOG_LIBRARIES}
  ${gflags_LIBRARIES}
  ${ABSL_LIBRARIES}
  ${PROTOBUF_LIBRARIES})
if(PNC_SELF_LIB)
  target_link_libraries(
      ${PROJECT_NAME}
      europa_self_common)
else()
  target_link_libraries(
    ${PROJECT_NAME}
    europa_common)
endif(PNC_SELF_LIB)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION .)
