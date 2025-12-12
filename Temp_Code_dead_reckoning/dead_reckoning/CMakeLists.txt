cmake_minimum_required(VERSION 3.12.1)
project(europa_dead_reckoning)
#add_definitions(-DMODULE_NAME="${PROJECT_NAME}")

include_directories(/usr/include/eigen3)
include_directories(${CMAKE_SOURCE_DIR})


project(europa_dead_reckoning)
# Minimal build option: when ON, generate object target and skip linking external libs.
option(BUILD_MINIMAL "Build minimal objects without linking external libraries" ON)

set(EUROPA_SRC
  common/kalmanfilter.cc
  common/structs.h
  common/util.cc
  src/dead_reckoning_core.cc
  src/quat_utils.cc
  src/dead_reckoning_gflags.cc)

if(BUILD_MINIMAL)
  add_library(${PROJECT_NAME}_objects OBJECT ${EUROPA_SRC})
  # Add a simple example executable that links the object files for local testing
  add_executable(simple_run examples/simple_run.cpp)
  # Link the object library's object files into the executable
  target_link_libraries(simple_run PRIVATE $<TARGET_OBJECTS:${PROJECT_NAME}_objects>)
else()
  add_library(${PROJECT_NAME} SHARED ${EUROPA_SRC})
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
  set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

  target_link_libraries(
    ${PROJECT_NAME}
    europa_common
    globalproto
    europa_hdmap
    ${ABSL_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    boost_thread
    gflags
    glog
    osqp
    protobuf)
endif()

if(NOT BUILD_MINIMAL)
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  manual_service DESTINATION .)
endif()

if(NOT BUILD_MINIMAL)
if(MIDDLEWARE MATCHES MDC)
  set(APP_NAME dead_reckoning)
  add_executable(${APP_NAME} main.cpp dead_reckoning.h dead_reckoning.cpp)
  set_target_properties(${APP_NAME} PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
  target_link_libraries(${APP_NAME} ${PROJECT_NAME} ${MDC_LINK_LIBS})
  install(
    TARGETS ${APP_NAME}
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION manual_service/${APP_NAME}/bin
    INCLUDES
    DESTINATION .)
  install(
    FILES run_dead_reckoning.sh
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    DESTINATION script/test)
  install(DIRECTORY ${CM_CONF}/location/locationProcess
          DESTINATION manual_service/${APP_NAME}/etc)
  install(FILES ${CM_CONF}/location/MANIFEST.json
          DESTINATION manual_service/${APP_NAME}/etc)
  install(FILES conf/dead_reckoning/dead_reckoning.yaml DESTINATION manual_service/${APP_NAME}/conf)
  install(DIRECTORY conf DESTINATION .)
endif()
endif()

if(NOT BUILD_MINIMAL)
if(MIDDLEWARE MATCHES CYBER)
  add_library(${PROJECT_NAME}_component SHARED dead_reckoning_component.cc)
  target_link_libraries(${PROJECT_NAME}_component ${PROJECT_NAME} fastrtps cyberrt)
  install(
    TARGETS ${PROJECT_NAME}_component
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION .)
  install(DIRECTORY conf dag launch DESTINATION .)
endif(MIDDLEWARE MATCHES CYBER)
endif()
