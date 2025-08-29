
cmake_minimum_required(VERSION 3.10.2)
project(perception_proto)

file(GLOB_RECURSE PROTOS  "*.proto")
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTOS})
include_directories(${PROTOBUF_INCLUDE_DIRS})
add_library(${PROJECT_NAME} SHARED ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME}
    cyber::cyber
    all_proto
    common
    perception_common)


install(
    TARGETS ${PROJECT_NAME} DESTINATION lib
)
