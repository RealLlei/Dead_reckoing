cmake_minimum_required(VERSION 3.12.1)
project(europa_prediction)

set(CMAKE_CXX_FLAGS "-Wall ${CMAKE_CXX_FLAGS}")
message("Prediction Compiler Options: " ${CMAKE_CXX_FLAGS})

file(GLOB
  prediction_common
  common/prediction_system_gflags.cc
  common/environment_features.cc
  common/message_process.cc
  common/prediction_util.cc
  common/prediction_thread_pool.cc
  common/feature_output.cc
  common/junction_analyzer.cc
  common/prediction_thread_pool.cc
  common/feature_output.cc
  common/junction_analyzer.cc
  common/prediction_gflags.cc
  common/prediction_map.cc
  common/road_graph.cc
  common/validation_checker.cc
  common/kalman_filter.cc
  common/kalman_motion_fusion.cc
  common/longitudinal_strategy.cc
  common/discretized_trajectory.cc
  container/pose/pose_container.cc
  container/container_manager.cc
  # container/storytelling/storytelling_container.cc
  container/obstacles/obstacles_container.cc
  container/obstacles/obstacle.cc
  container/obstacles/obstacle_clusters.cc
  container/adc_trajectory/adc_trajectory_container.cc
  # network/rnn_model/rnn_model.cc
  # network/net_model.cc
  # network/net_layer.cc
  # network/net_util.cc
  predictor/free_move/free_move_predictor.cc
  predictor/move_sequence/move_sequence_predictor.cc
  predictor/extrapolation/extrapolation_predictor.cc
  predictor/predictor.cc
  predictor/predictor_manager.cc
  predictor/cost/cost_predictor.cc
  predictor/dbn/dbn_predictor.cc
  predictor/dbn/base_dbn_model.cc
  predictor/dbn/dbn_junction_predictor.cc
  predictor/dbn/dbn_junction_model.cc
  predictor/sequence/sequence_predictor.cc
  # predictor/lane_sequence/lane_sequence_predictor.cc
  # predictor/interaction/interaction_predictor.cc
  # predictor/junction/junction_predictor.cc
  # predictor/single_lane/single_lane_predictor.cc
  predictor/empty/empty_predictor.cc
  predictor/follow_ego_lane/follow_urban_ego_lane_predictor.cc
  predictor/follow_ego_lane/follow_ego_lane_predictor.cc
  predictor/pedestrian/pedestrian_trajnet_predictor.cc
  predictor/vectornet/tnt_predictor.cc
  predictor/static/static_predictor.cc
  predictor/ego/self_predictor.cc
  scenario/scenario_manager.cc
  # scenario/right_of_way/right_of_way.cc
  scenario/prioritization/obstacles_prioritizer.cc
  scenario/perception_filter/sensor_analyzer.h
  scenario/perception_filter/perception_filter.h
  # scenario/analyzer/scenario_analyzer.cc
  # scenario/feature_extractor/feature_extractor.cc
  # scenario/scenario_features/junction_scenario_features.cc
  # scenario/scenario_features/scenario_features.cc
  # scenario/scenario_features/cruise_scenario_features.cc
  util/data_extraction.cc
  prediction.cc
  # submodules/submodule_output.cc
  pipeline/vector_net.cc
  inference_manager.h
)

if(PLATFORM MATCHES "orin")
  add_library(
    europa_prediction SHARED
    predictor/vectornet/tnt_orin_inference.h
    predictor/qcnet/qcnet_orin_inference.h
    predictor/qcnet/qcnet_predictor.cc
    ${prediction_common}
  )
  set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

  target_link_libraries(
    europa_prediction
    globalproto
    nvdla_compiler
    nvinfer
    nvinfer_plugin
    nvonnxparser
    cudla
    cublas
    cublasLt
    cudnn
    ${ABSL_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    # ${TORCH_LIBRARIES}
    # ${PYTHON_LIBRARIES}
    # ${ORT_LIBRARIES}
  )

  install(TARGETS ${PROJECT_NAME} DESTINATION ./bin)
endif()

if(PLATFORM MATCHES "mdc")
  # arm
  add_library(
    europa_prediction SHARED
    predictor/vectornet/tnt_om_inference.h

    ${prediction_common}
  )
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "-Wl,--disable-new-dtags")
  set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
  target_link_libraries(europa_prediction
    globalproto
    ${ABSL_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${VISUAL_LIBRARIES}
    ${ADB_LIBRARIES}

    # ${HZ_DATA_COLLECT_CLIENT_LIBRARIES}
    ${ACLLIB_LIBRARIES}
    ${ARA_LIBRARIES})
endif(PLATFORM MATCHES "mdc")

if(PLATFORM MATCHES "x86")
  file(GLOB_RECURSE RPROTO "proto/*.cc")
  add_library(
    europa_prediction SHARED
    ${RPROTO}
    predictor/vectornet/tnt_ort_inference.h
    predictor/qcnet/qcnet_ort_inference.h
    predictor/qcnet/qcnet_predictor.cc

    ${prediction_common}
  )
  set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

  target_link_libraries(
    europa_prediction
    globalproto
    ${ABSL_LIBRARIES}
    ${BOOST_LIBRARIES}
    ${GLOG_LIBRARIES}
    ${GFLAGS_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${TORCH_LIBRARIES}
    ${PYTHON_LIBRARIES}
    ${ORT_LIBRARIES}
  )
endif(PLATFORM MATCHES "x86")

if(PNC_SELF_LIB)
  target_link_libraries(
      ${PROJECT_NAME}
      europa_self_hdmap
      europa_self_common)
else()
  target_link_libraries(
    ${PROJECT_NAME}
    europa_hdmap
    europa_common)
endif(PNC_SELF_LIB)

# lib prediction
if(TARGET europa_prediction)
  set_target_properties(europa_prediction PROPERTIES LINKER_LANGUAGE CXX)
  install(
    TARGETS europa_prediction
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES
    DESTINATION .)
  install(DIRECTORY conf/ DESTINATION ./conf/prediction)
  install(DIRECTORY model DESTINATION ./data)
endif()