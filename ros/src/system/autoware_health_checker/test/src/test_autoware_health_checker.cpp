/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *
 * v1.0 Masaya Kataoka
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <autoware_health_checker/node_status_publisher.h>

class AutowareHealthCheckerTestSuite : public ::testing::Test
{
public:
  AutowareHealthCheckerTestSuite()
  {
  }

  ~AutowareHealthCheckerTestSuite()
  {
  }
};

class AutowareHealthCheckerTestClass
{
public:
  AutowareHealthCheckerTestClass()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    node_status_publisher_ = std::make_shared<autoware_health_checker::NodeStatusPublisher>(nh,pnh);
  };
  std::shared_ptr<autoware_health_checker::NodeStatusPublisher> node_status_publisher_;
};

TEST(TestSuite, CheckOnePoint)
{
  AutowareHealthCheckerTestClass test_autoware_health_checker;
  uint8_t ret_fatal = test_autoware_health_checker.node_status_publisher_->CHECK_MIN_VALUE("test",0,3,2,1,"test");
  ASSERT_EQ(ret_fatal, autoware_health_checker::LEVEL_FATAL) << "The value was self-diagnosed as fatal"; // minimum point is 2
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "AutowareHealthCheckerTestNode");
  return RUN_ALL_TESTS();
}