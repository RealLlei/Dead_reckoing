cmake_minimum_required(VERSION 3.12.1)
project(europa_self_simulator)

string(REPLACE " --coverage" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})

set(CMAKE_CXX_FLAGS "-w ${CMAKE_CXX_FLAGS}")
message("Self_Simulator Compiler Options: " ${CMAKE_CXX_FLAGS})

add_library(${PROJECT_NAME} SHARED
  sim_dummy_obs/base_sim_data.cc
  sim_dummy_obs/common/util_com.cc
  sim_dummy_obs/obs_manager.cc
  sim_dummy_obs/obstacle/prediction_obs/prediction_obstacle_base.cc
  sim_dummy_obs/obstacle/perception_obs/side_lane_obs_2.cc
  sim_dummy_obs/obstacle/perception_obs/side_lane_obs.cc
  sim_dummy_obs/obstacle/perception_obs/front_obs.cc
  sim_dummy_obs/obstacle/perception_obs/opposite_lane_obs_1.cc
  sim_dummy_obs/obstacle/perception_obs/perception_obstacle_base.cc
  sim_dummy_obs/obstacle/perception_obs/relative_position_obs.cc
  sim_dummy_obs/obstacle/perception_obs/lane_follow_obs.cc
  sim_dummy_obs/obstacle/dummy_obstacle_base.cc
  sim_dummy_obs/obstacle/planning_obs/front_obs_planning.cc
  sim_dummy_obs/obstacle/planning_obs/planning_obstacle_base.cc
  sim_control/sim_control.cc
  sim_dummy_gflags.cc
  self_simulator.cc
)
set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION .)
install(DIRECTORY self_simulator_conf DESTINATION ./conf/planning)
