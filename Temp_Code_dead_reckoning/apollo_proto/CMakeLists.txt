cmake_minimum_required(VERSION 3.12.1)
if ( NOT FROM_TOP )
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

  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  message("Total Compiler Options: " ${CMAKE_CXX_FLAGS})
  project(europa_common_proto)

  message(STATUS ${PROJECT_NAME} " PLATFORM: " ${PLATFORM} " CMAKE_BUILD_TYPE: " ${CMAKE_BUILD_TYPE})
  set(MDC_SDK_INCLUDE "${MDC_SDK}/sysroot/usr/include/")
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/release/cyber)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
  message((STATUS "CMAKE_INSTALL_PREFIX: " ${CMAKE_INSTALL_PREFIX}))

  include(../third_party/cmake/Packages.cmake)
  find_package(Protobuf REQUIRED)
  find_program(CCACHE_FOUND ccache)
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
endif()

link_directories(${CMAKE_INSTALL_PREFIX}/lib)

SET(CMAKE_CXX_FLAGS "-w ${CMAKE_CXX_FLAGS}")
message("Proto Compiler Options: " ${CMAKE_CXX_FLAGS})
project(europa_common_proto)

file(
  GLOB
  PROTOS
  "localization/*.proto"
  "common/*.proto"
  "canbus/*.proto"
  "map/*.proto"
  "control/*.proto"
  "dead_reckoning/*.proto"
  "drivers/*.proto"
  "hmi/*.proto"
  "dreamview/*.proto"
  "planning/*.proto"
  "planning/math/*.proto"
  "perception/*.proto"
  "prediction/*.proto"
  "routing/*.proto"
  "storytelling/*.proto"
  "transform/*.proto"
  "fsm/*.proto"
  "sensors/*.proto"
  )
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTOS})

if (BUILD_BY_ADF)
    message("Build by adf")
    set(LIB_OUTPUT_DIR "${CMAKE_SOURCE_DIR}/output/${TARGET_PLATFORM}/lib")
    set(3RD_PROTO_LIB "${CMAKE_SOURCE_DIR}/third_party/${TARGET_PLATFORM}/protobuf/lib/libprotobuf.so")
elseif (BUILD_BY_PERCEPTION)
    message("Build by perception")
    set(3RD_PROTO_LIB "${THIRDPARTY_ROOT}/${TARGET_PLATFORM}/protobuf/lib/libprotobuf.so")
else()
    set(LIB_OUTPUT_DIR "lib")
    set(3RD_PROTO_LIB ${PROTOBUF_LIBRARIES})
endif()

add_library(${PROJECT_NAME} SHARED ${PROTO_SRCS})
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME} ${3RD_PROTO_LIB})

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${LIB_OUTPUT_DIR}
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION .)
