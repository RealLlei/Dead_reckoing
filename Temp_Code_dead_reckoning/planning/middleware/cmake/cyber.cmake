add_library(${PROJECT_NAME}_component SHARED middleware/cyber/planning_component.cc)
target_link_libraries(${PROJECT_NAME}_component 
  ${PROJECT_NAME} 
  fastrtps 
  cyberrt
  europa_common_proto)

find_package(GTEST REQUIRED)
add_executable(convert_tool_test middleware/cyber/convert_tool_test.cc)
target_link_libraries(convert_tool_test
 ${PROJECT_NAME}
 europa_common_proto
 globalproto
 ${GTEST_LIBRARIES}
)

if(COMPILE_ALL AND (LLVM MATCHES 0))
  target_link_libraries(
    ${PROJECT_NAME}
    gcov)
endif()

install(
  TARGETS ${PROJECT_NAME}_component convert_tool_test
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION .)
install(PROGRAMS run_planning DESTINATION bin)
install(DIRECTORY dag launch DESTINATION .)
