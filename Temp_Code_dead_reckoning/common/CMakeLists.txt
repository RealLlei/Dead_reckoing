cmake_minimum_required(VERSION 3.12.1)

if(PNC_SELF_LIB)
  project(europa_self_common)
else()
  project(europa_common)
endif(PNC_SELF_LIB)

if((PLATFORM MATCHES "x86" OR PLATFORM MATCHES "20dv") AND (COMPILE_ALL))
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftest-coverage -fprofile-arcs")
endif()

if( NOT FROM_TOP ) 
  message("NOT FROM_TOP")
  if(PLATFORM MATCHES "mdc")
    if(LLVM MATCHES 1)
      set(MDC_SDK "/usr/local/mdc_sdk_llvm/dp_gea/mdc_cross_compiler/")
      set(CMAKE_C_COMPILER ${MDC_SDK}/bin/clang)
      set(CMAKE_CXX_COMPILER ${MDC_SDK}/bin/clang++)
      set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
      set(llvm_config 1)
    else()
      set(MDC_SDK "/usr/local/mdc_sdk/dp_gea/mdc_cross_compiler/")
      set(CMAKE_C_COMPILER ${MDC_SDK}/bin/aarch64-target-linux-gnu-gcc)
      set(CMAKE_CXX_COMPILER ${MDC_SDK}/bin/aarch64-target-linux-gnu-g++)
      set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
    endif()
  else()
    if(LLVM MATCHES 1)
      message("USES Clang-10")
      set(CMAKE_CXX_COMPILER /usr/bin/clang++-10)
      set(CMAKE_C_COMPILER /usr/bin/clang-10)
      set(llvm_config 1)
    endif()
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
  endif()
  
  if (llvm_config MATCHES 1)
    if (CMAKE_BUILD_TYPE MATCHES debug)
      set(CMAKE_CXX_FLAGS "-fPIC -O0 -g -Wall -Wextra -Werror=return-type ${CMAKE_CXX_FLAGS}")
    elseif(CMAKE_BUILD_TYPE MATCHES release)
      set(CMAKE_CXX_FLAGS "-fPIC -O3 -g -Wall -Werror=return-type ${CMAKE_CXX_FLAGS}")
    endif()
  else()
    if(CMAKE_BUILD_TYPE MATCHES debug)
      set(CMAKE_CXX_FLAGS "-fPIC -O0 -g -Wextra -Wall -fmessage-length=0 -fdiagnostics-color=always ${CMAKE_CXX_FLAGS}")
    elseif(CMAKE_BUILD_TYPE MATCHES release)
      set(CMAKE_CXX_FLAGS "-fPIC -O3 -g -fmessage-length=0 -fdiagnostics-color=always ${CMAKE_CXX_FLAGS}")
    endif()
  endif()
  
  message("Total Compiler Options: " ${CMAKE_CXX_FLAGS})
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  
  message(STATUS ${PROJECT_NAME} " PLATFORM: " ${PLATFORM} " CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})
  set(MDC_SDK_INCLUDE "${MDC_SDK}/sysroot/usr/include/")
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/release/cyber)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
  message((STATUS "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX}))
  
  include(../third_party/cmake/Packages.cmake)
  find_package(Cyber REQUIRED)
  find_package(ORT REQUIRED)
  find_package(Json REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(Ffmpeg REQUIRED)
  find_package(Civetweb REQUIRED)
  find_package(Protobuf REQUIRED)
  find_package(Gflags REQUIRED)
  find_package(GLOG REQUIRED)
  find_package(Boost REQUIRED)
  find_package(OSQP REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(Absl REQUIRED)
  find_package(ZMQ REQUIRED)
  find_package(GTEST REQUIRED)
  find_package(CppAD REQUIRED)
  find_package(Sqlite3 REQUIRED)
  find_package(OpenCV REQUIRED)
  
  if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
  endif(CCACHE_FOUND)
  
  if(PLATFORM MATCHES "mdc")
    add_definitions(-DISMDC="is mdc")
    set(CMAKE_SYSTEM_NAME Linux)
    set(CMAKE_SYSTEM_PROCESSOR arm)
  
    set(CMAKE_FIND_ROOT_PATH ${MDC_SDK}/sysroot/ ${CMAKE_FIND_ROOT_PATH})
  
    set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/release/pnc_mdc)
    add_definitions(-DMDC_COMPILER_AOS_SEA_LLVM)
    add_definitions(-DMDC_PRODUCTION_MDC610)
    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
  
    set(CMAKE_C_COMPILER_WORKS 1)
    set(CMAKE_CXX_COMPILER_WORKS 1)
    set(AP_ADSF ${MDC_ADSFI_SDK}/adsf)
    set(CMAKE_INSTALL_RPATH "/opt/app/1/lib")
  
    # 用户需要添加自己的头文件路径
    include_directories(
      ${MDC_ADSFI_SDK}
      ${YAML_SDK}
      ${MDC_ARXML_SDK}
      ${AP_ADSF}
      ${MDC_COLLECT_SDK}
      ${MDC_HEALTH_SDK}
      ${MDC_SDK}/sysroot/usr/include/yaml-cpp
      ${MDC_ACLLIB_SDK}/include
      ${MDC_ACLLIB_SDK}/include/acl
      ${MDC_ACLLIB_SDK}/include/acl/ops
      ${PNC_ROOT}/ap-release/include
      ../)
  endif()
  link_directories(${CMAKE_CURRENT_SOURCE_DIR}/../proto/build)
endif()

if(PLATFORM MATCHES "orin")
  message("BUILD ORIN")
  add_definitions(-DISORIN="is orin")
endif()

include_directories(
      ../)

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

SET(CMAKE_CXX_FLAGS "-w ${CMAKE_CXX_FLAGS}")
message("Common Compiler Options: " ${CMAKE_CXX_FLAGS})


add_library(
  ${PROJECT_NAME} SHARED
  adapters/adapter_gflags.cc
  configs/config_gflags.cc
  configs/vehicle_config_helper.cc
  file/file.cc
  file/log.h
  file/global_data.cc
  file/types.h
  filters/digital_filter.cc
  filters/digital_filter_coefficients.cc
  filters/mean_filter.cc
  filters/recursive_least_square_estimator.cc
  interpolation/interpolation_1d.cc
  interpolation/interpolation_2d.cc
  kv_db/kv_db.cc
  math/linear_quadratic_regulator.cc
  math/box2d.cc
  math/search.cc
  math/aabox2d.cc
  math/path_matcher.cc
  math/matrix_operations.cc
  math/math_utils.cc
  math/vec2d.cc
  math/cartesian_frenet_conversion.cc
  math/angle.cc
  math/line_segment2d.cc
  math/integral.cc
  math/polygon2d.cc
  math/linear_interpolation.cc
  math/double_type.h
  math/kalman_filter.h
  math/pure_pursuit_control.cc
  pb2struct/pb2struct.h
  struct2pb/struct2pb.h
  status/status.h
  time/clock.cc
  time/duration.cc
  time/time.cc
  util/eigen_defs.h
  util/factory.h
  util/json_util.cc
  util/lru_cache.h
  util/macros.h
  util/perf_util.cc
  util/point_factory.h
  util/points_downsampler.h
  util/string_util.cc
  util/time_util.h
  util/util.cc
  util/base/macros.h
  util/base/bounded_queue.h
  util/base/wait_strategy.h
  utm_projection/coordinate_convertor.cc
  utm_projection/utm_zone.cc
  vehicle_model/vehicle_model.cc
  vehicle_state/vehicle_state_provider.cc
  thread/thread_pool.cc
  zmq_conventor/zmq_receiver.cc
  zmq_conventor/zmq_sender.cc)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

if(ENABLE_ASAN AND (PLATFORM MATCHES "x86"))
  list(FILTER BOOST_LIBRARIES EXCLUDE REGEX "libboost_python27.so")
endif()

target_link_libraries(
  ${PROJECT_NAME}
  ${BOOST_LIBRARIES}
  ${OSQP_LIBRARIES}
  ${ABSL_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${GFLAGS_LIBRARIES}
  ${GLOG_LIBRARIES}
  ${ZMQ_LIBRARIES}
  ${Sqlite3_LIBRARIES})

  if(COMPILE_ALL AND (LLVM MATCHES 0))
    target_link_libraries(
      ${PROJECT_NAME}
      gcov)
  endif()

if((PLATFORM MATCHES "x86") OR (PLATFORM MATCHES "20dv") OR (PLATFORM MATCHES "baidu"))
#   add_library(${PROJECT_NAME}_cyber_patch SHARED
#               kv_db/kv_db.cc kv_db/kv_db_tool.cc util/json_util.cc)
#   target_link_libraries(${PROJECT_NAME}_cyber_patch sqlite3)
  target_link_libraries(${PROJECT_NAME} cyberrt)
#   install(TARGETS ${PROJECT_NAME}_cyber_patch DESTINATION lib)
#   install(DIRECTORY scripts DESTINATION .)
endif()

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION .)

install(FILES data/global_flagfile.txt
              data/vehicle_param/ep30/vehicle_param.pb.txt
              vehicle_model/conf/vehicle_model_config.pb.txt DESTINATION conf)

install(DIRECTORY data/vehicle_param DESTINATION conf)
