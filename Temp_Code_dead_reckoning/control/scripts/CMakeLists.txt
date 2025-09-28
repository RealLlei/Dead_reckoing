cmake_minimum_required(VERSION 3.12.1)

project(mbd_control)
file(GLOB MBDSRC "./*")

add_library(
  ${PROJECT_NAME} SHARED
  ${MBDSRC}
)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION .)
