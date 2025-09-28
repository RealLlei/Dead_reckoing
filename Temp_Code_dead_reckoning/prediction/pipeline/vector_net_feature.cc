/******************************************************************************
 * Copyright 2021 The Apollo Authors. All Rights Reserved.
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

#include "planning/prediction/pipeline/vector_net.h"

using TL::prediction::VectorNet;

int main(int argc, char* argv[]) {
  if (argc != 6) {
    AERROR << "usage: getVectorMap x y theta file_name --map_dir=data/map/demo";
  }
  google::ParseCommandLineFlags(&argc, &argv, true);
  VectorNet vector_net = VectorNet();
  double x = atof(argv[1]);
  double y = atof(argv[2]);
  double heading = atof(argv[3]);
  std::string file_name = static_cast<std::string>(argv[4]);
  std::cout << "begin create vector map for" << x << "," << y << "," << heading
            << ", to " << file_name << std::endl;

  vector_net.offline_query(x, y, heading, file_name);
  return 0;
}
