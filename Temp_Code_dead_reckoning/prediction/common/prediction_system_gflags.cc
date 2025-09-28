/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "planning/prediction/common/prediction_system_gflags.h"

#include <limits>

// System gflags
DEFINE_string(prediction_module_name, "prediction",
              "Default prediction module name");
DEFINE_string(prediction_conf_file,
              "conf/prediction/prediction_rule_conf.pb.txt",
              "Default conf file for prediction");
DEFINE_string(prediction_adapter_config_filename,
              "conf/prediction/adapter.conf",
              "Default conf file for prediction");
DEFINE_string(prediction_data_dir, "data/prediction/",
              "Prefix of files to store feature data");
DEFINE_string(offline_feature_proto_file_name, "",
              "The bin file including a series of feature proto messages");
DEFINE_string(output_filename, "", "The filename for offline process.");
DEFINE_string(extract_feature_type, "",
              "The extract feature type, either cruise or junction");

DEFINE_bool(prediction_test_mode, false, "Set prediction to test mode");
DEFINE_double(
    prediction_test_duration, std::numeric_limits<double>::infinity(),
    "The runtime duration in test mode (in seconds). Negative value will not "
    "restrict the runtime duration.");

DEFINE_string(
    prediction_offline_bags, "",
    "a list of bag files or directories for offline mode. The items need to be "
    "separated by colon ':'.  If this value is not set, the prediction module "
    "will use the listen to published ros topic mode.");
DEFINE_int32(prediction_offline_mode, 0,
             "0: online mode, no dump file"
             "1: dump feature proto to feature.*.bin"
             "2: dump data for learning to datalearn.*.bin"
             "3: dump predicted trajectory to predict_result.*.bin"
             "4: dump frame environment info to frame_env.*.bin"
             "5: dump data for tuning to datatuning.*.bin");
DEFINE_bool(enable_online_record4Prediction, false,
            "If enable record data for prediction.");
DEFINE_bool(enable_multi_thread, false, "If enable multi-thread.");
DEFINE_int32(max_thread_num, 8, "Maximal number of threads.");
DEFINE_int32(max_caution_thread_num, 2,
             "Maximal number of threads for caution obstacles.");
DEFINE_bool(enable_async_draw_base_image, true,
            "If enable async to draw base image");
DEFINE_bool(use_cuda, true, "If use cuda for torch.");
DEFINE_bool(enable_prediction_debug, true, "if enable debug prediction info");
DEFINE_int32(prediction_debug_history_frame_num, 30,
             "The number of history frames to debug");
DEFINE_double(offline_mode_start_timestamp, -1.0,
              "offline mode replay start timestamp");
DEFINE_double(offline_mode_end_timestamp, -1.0,
              "offline mode replay end timestamp");

// Bag replay timestamp gap
DEFINE_double(replay_timestamp_gap, 10.0,
              "Max timestamp gap for rosbag replay");
DEFINE_int32(max_num_dump_feature, 10000, "Max number of features to dump");
DEFINE_int32(max_num_dump_dataforlearn, 5000,
             "Max number of dataforlearn to dump");

// Submodules
DEFINE_bool(use_lego, false, "If use lego architecture");
DEFINE_string(container_topic_name, "/apollo/prediction/container",
              "Container topic name");
DEFINE_string(adccontainer_topic_name, "/apollo/prediction/adccontainer",
              "ADC Container topic name");
DEFINE_string(evaluator_topic_name, "/apollo/prediction/evaluator",
              "Evaluator topic name");
DEFINE_string(container_submodule_name, "container_submodule",
              "Container submodule name");
DEFINE_string(evaluator_submodule_name, "evaluator_submodule",
              "Evaluator submodule name");
DEFINE_string(perception_obstacles_topic_name,
              "/apollo/prediction/perception_obstacles",
              "Internal topic of perception obstacles");
