cmake_minimum_required(VERSION 3.12.1)
project(europa_control)
#add_definitions(-DMODULE_NAME="${PROJECT_NAME}")

add_subdirectory(mbd_control/mbd_src)
# file(GLOB MBDSRC "mbd_control/mbd_src/*")
file(
  GLOB
  PROTOS
  "proto/*.proto"
  )
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOS})

add_library(
  ${PROJECT_NAME} SHARED
  ${PROTO_SRCS}
  core/control_core.cc
  common/control_gflags.cc
  common/interpolation_1d.cc
  common/leadlag_controller.cc
  common/trajectory_analyzer.cc
  common/pid_BC_controller.cc
  common/interpolation_2d.cc
  common/pid_IC_controller.cc
  common/mrac_controller.cc
  common/hysteresis_filter.cc
  common/pid_controller.cc
  controller/controller.h
  controller/mpc_controller.cc
  controller/lat_controller.cc
  controller/controller_agent.cc
  controller/lon_controller.cc
  mbd_control/mbd_control.cc
  mbd_control/pb_struct.cc
  mbd_control/mbd_control_gflags.cc
  )
target_link_libraries(
  ${PROJECT_NAME}
  mbd_control
  europa_common
  europa_common_proto
  globalproto
  ${ABSL_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${OSQP_LIBRARIES}
  )
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION .)
install(DIRECTORY conf DESTINATION .)

if(MIDDLEWARE MATCHES MDC)
  set(APP_NAME control)
  add_executable(
    ${APP_NAME} main.cpp control.h control.cpp teleop.h)
  target_link_libraries(${APP_NAME} ${PROJECT_NAME} ${VISUAL_LIBRARIES}
                        ${ADB_LIBRARIES} ${ACLLIB_LIBRARIES} ${ARA_LIBRARIES})

  install(
    TARGETS ${APP_NAME}
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION runtime_service/${APP_NAME}/bin
    INCLUDES
    DESTINATION .)
  install(
    FILES run_control.sh
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    DESTINATION script/test)
  install(DIRECTORY ${CM_CONF}/TLExecutable_Control/TLProcess_Control
          DESTINATION runtime_service/${APP_NAME}/etc)
  install(FILES ${CM_CONF}/TLExecutable_Control/MANIFEST.json
          DESTINATION runtime_service/${APP_NAME}/etc)
  install(FILES conf/control_config.yaml DESTINATION runtime_service/${APP_NAME}/conf)
endif()

if(PLATFORM MATCHES "x86")
  add_library(${PROJECT_NAME}_component SHARED control_component.cc)
  target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME} fastrtps cyberrt)
  install(
    TARGETS ${PROJECT_NAME}_component
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION .)
  install(DIRECTORY dag launch DESTINATION .)
endif(PLATFORM MATCHES "x86")
