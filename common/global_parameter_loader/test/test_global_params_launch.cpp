// Copyright 2023 The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <cstdlib>
#include <iostream>
#include <string>

TEST(TestLaunchFile, test_launch_file)
{
  // Define the path of Python launch file
  std::string global_params_launch_path = "global_params.launch.py";

  // Define the parameters you want to pass to the launch file
  std::string use_sim_time_param = "false";
  std::string vehicle_model_param = "sample_vehicle";
  // Construct the command to run the Python launch script with parameters
  std::string command = "ros2 launch global_parameter_loader " + global_params_launch_path +
                        " use_sim_time:=" + use_sim_time_param +
                        " vehicle_model:=" + vehicle_model_param;

  // Use the system() function to execute the command
  int result = std::system(command.c_str());
  // Check the result of running the launch file
  EXPECT_EQ(result, 0);
}

int main(int argc, char * argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
