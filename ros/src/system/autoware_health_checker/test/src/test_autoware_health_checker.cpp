/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
#include <autoware_health_checker/node_status_publisher.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

class AutowareHealthCheckerTestSuite : public ::testing::Test {
public:
  AutowareHealthCheckerTestSuite() {}

  ~AutowareHealthCheckerTestSuite() {}
};

class AutowareHealthCheckerTestClass {
public:
  AutowareHealthCheckerTestClass() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    node_status_publisher_ptr =
        std::make_shared<autoware_health_checker::NodeStatusPublisher>(nh, pnh);
  };
  std::shared_ptr<autoware_health_checker::NodeStatusPublisher>
      node_status_publisher_ptr;
};

TEST(TestSuite, CHECK_MIN_VALUE) {
  AutowareHealthCheckerTestClass test_autoware_health_checker;
  uint8_t ret_fatal =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_MIN_VALUE(
          "test", 1, 6, 4, 2, "test");
  ASSERT_EQ(ret_fatal, autoware_health_checker::LEVEL_FATAL)
      << "The value was self-diagnosed as fatal";
  uint8_t ret_error =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_MIN_VALUE(
          "test", 3, 6, 4, 2, "test");
  ASSERT_EQ(ret_error, autoware_health_checker::LEVEL_ERROR)
      << "The value was self-diagnosed as error";
  uint8_t ret_warn =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_MIN_VALUE(
          "test", 5, 6, 4, 2, "test");
  ASSERT_EQ(ret_warn, autoware_health_checker::LEVEL_WARN)
      << "The value was self-diagnosed as warn";
  uint8_t ret_ok =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_MIN_VALUE(
          "test", 7, 6, 4, 2, "test");
  ASSERT_EQ(ret_ok, autoware_health_checker::LEVEL_OK)
      << "The value was self-diagnosed as ok";
}

TEST(TestSuite, CHECK_MAX_VALUE) {
  AutowareHealthCheckerTestClass test_autoware_health_checker;
  uint8_t ret_fatal =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_MAX_VALUE(
          "test", 7, 2, 4, 6, "test");
  ASSERT_EQ(ret_fatal, autoware_health_checker::LEVEL_FATAL)
      << "The value was self-diagnosed as fatal";
  uint8_t ret_error =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_MAX_VALUE(
          "test", 5, 2, 4, 6, "test");
  ASSERT_EQ(ret_error, autoware_health_checker::LEVEL_ERROR)
      << "The value was self-diagnosed as error";
  uint8_t ret_warn =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_MAX_VALUE(
          "test", 3, 2, 4, 6, "test");
  ASSERT_EQ(ret_warn, autoware_health_checker::LEVEL_WARN)
      << "The value was self-diagnosed as warn";
  uint8_t ret_ok =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_MAX_VALUE(
          "test", 1, 2, 4, 6, "test");
  ASSERT_EQ(ret_ok, autoware_health_checker::LEVEL_OK)
      << "The value was self-diagnosed as ok";
}

TEST(TestSuite, CHECK_RANGE) {
  AutowareHealthCheckerTestClass test_autoware_health_checker;
  uint8_t ret_fatal =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_RANGE(
          "test", 7.0, {2.0, 4.0}, {1.0, 5.0}, {0.0, 6.0}, "test");
  ASSERT_EQ(ret_fatal, autoware_health_checker::LEVEL_FATAL)
      << "The value was self-diagnosed as fatal";
  uint8_t ret_error =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_RANGE(
          "test", 5.5, {2.0, 4.0}, {1.0, 5.0}, {0.0, 6.0}, "test");
  ASSERT_EQ(ret_error, autoware_health_checker::LEVEL_ERROR)
      << "The value was self-diagnosed as error";
  uint8_t ret_warn =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_RANGE(
          "test", 4.5, {2.0, 4.0}, {1.0, 5.0}, {0.0, 6.0}, "test");
  ASSERT_EQ(ret_warn, autoware_health_checker::LEVEL_WARN)
      << "The value was self-diagnosed as warn";
  uint8_t ret_ok =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_RANGE(
          "test", 3.0, {2.0, 4.0}, {1.0, 5.0}, {0.0, 6.0}, "test");
  ASSERT_EQ(ret_ok, autoware_health_checker::LEVEL_OK)
      << "The value was self-diagnosed as ok";
}

uint8_t test_function(double value) {
  if (value == 0.0) {
    return autoware_health_checker::LEVEL_FATAL;
  }
  if (value == 1.0) {
    return autoware_health_checker::LEVEL_ERROR;
  }
  if (value == 2.0) {
    return autoware_health_checker::LEVEL_WARN;
  }
  return autoware_health_checker::LEVEL_OK;
};

boost::property_tree::ptree test_value_json_func(double value) {
  boost::property_tree::ptree tree;
  tree.put("value", value);
  return tree;
};

TEST(TestSuite, CHECK_VALUE) {
  AutowareHealthCheckerTestClass test_autoware_health_checker;
  std::function<uint8_t(double value)> check_func = test_function;
  std::function<boost::property_tree::ptree(double value)>
      check_value_json_func = test_value_json_func;
  uint8_t ret_fatal =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_VALUE(
          "test", 0.0, check_func, check_value_json_func, "test");
  ASSERT_EQ(ret_fatal, autoware_health_checker::LEVEL_FATAL)
      << "The value was self-diagnosed as fatal";
  uint8_t ret_error =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_VALUE(
          "test", 1.0, check_func, check_value_json_func, "test");
  ASSERT_EQ(ret_error, autoware_health_checker::LEVEL_ERROR)
      << "The value was self-diagnosed as fatal";
  uint8_t ret_warn =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_VALUE(
          "test", 2.0, check_func, check_value_json_func, "test");
  ASSERT_EQ(ret_warn, autoware_health_checker::LEVEL_WARN)
      << "The value was self-diagnosed as fatal";
  uint8_t ret_ok =
      test_autoware_health_checker.node_status_publisher_ptr->CHECK_VALUE(
          "test", -1.0, check_func, check_value_json_func, "test");
  ASSERT_EQ(ret_ok, autoware_health_checker::LEVEL_OK)
      << "The value was self-diagnosed as fatal";
  boost::optional<double> value =
      check_value_json_func(0.0).get_optional<double>("value");
  ASSERT_EQ(value.get(), 0.0)
      << "The value must be true, failed to get json value";
}

TEST(TestSuite, NODE_STATUS) {
  AutowareHealthCheckerTestClass test_autoware_health_checker;
  test_autoware_health_checker.node_status_publisher_ptr->NODE_ACTIVATE();
  uint8_t ret_active =
      test_autoware_health_checker.node_status_publisher_ptr->getNodeStatus();
  ASSERT_EQ(ret_active, true) << "The value must be true";
  test_autoware_health_checker.node_status_publisher_ptr->NODE_DEACTIVATE();
  uint8_t ret_inactive =
      test_autoware_health_checker.node_status_publisher_ptr->getNodeStatus();
  ASSERT_EQ(ret_inactive, false) << "The value must be true";
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "AutowareHealthCheckerTestNode");
  return RUN_ALL_TESTS();
}