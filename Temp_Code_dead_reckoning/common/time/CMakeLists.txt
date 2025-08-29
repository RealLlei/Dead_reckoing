cmake_minimum_required(VERSION 3.12.1)

add_library(cyber_time duration.cc clock.cc rate.cc time.cc)
target_link_libraries(cyber_time cyber::common)
add_library(cyber::time ALIAS cyber_time)

if(MAKE_TEST)
  add_executable(duration_test duration_test.cc)
  target_link_libraries(duration_test gtest gtest_main cyber::time)

  add_executable(time_test time_test.cc)
  target_link_libraries(time_test gtest gtest_main cyber::time cyber::common)
  install(TARGETS duration_test time_test RUNTIME DESTINATION bin DESTINATION .)
endif()

install(
  TARGETS cyber_time
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION .)
