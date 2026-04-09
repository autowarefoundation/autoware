// Copyright 2021 Tier IV, Inc.
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

#include "roboteq_serial_interface/roboteq_serial_interface.hpp"

#include <boost/exception/all.hpp>

#include <regex>
#include <thread>

RoboteqSerialInterface::RoboteqSerialInterface(const rclcpp::NodeOptions & node_options)
: Node("roboteq_serial_interface", node_options), serial_(io_)
{
  using std::placeholders::_1;

  // Subscriber
  command_sub_ = this->create_subscription<roboteq_msgs::msg::RoboteqCommandStamped>(
    "~/input/command", 10, std::bind(&RoboteqSerialInterface::onRoboteqCommand, this, _1));

  // Publisher
  status_pub_ =
    this->create_publisher<roboteq_msgs::msg::RoboteqStatusStamped>("~/output/status", 10);

  // serial
  devname_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
  openSerial();
  std::thread read_thr(&RoboteqSerialInterface::readSerial, this);
  read_thr.detach();
}

void RoboteqSerialInterface::openSerial()
{
  using boost::asio::serial_port_base;

  if (!serial_.is_open()) {
    try {
      serial_.open(devname_);
      serial_.set_option(serial_port_base::baud_rate(115200));
      serial_.set_option(serial_port_base::character_size(8));
      serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
      serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
      serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
      RCLCPP_INFO_STREAM(this->get_logger(), "open serial");
    } catch (boost::wrapexcept<boost::system::system_error> & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), __FUNCTION__ << ": " << e.what());
      std::this_thread::sleep_for(std::chrono::microseconds(5000));
      serial_.close();
    }
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "already opened");
  }
}

void RoboteqSerialInterface::closeSerial()
{
  if (serial_.is_open()) {
    try {
      serial_.close();
      RCLCPP_INFO_STREAM(this->get_logger(), "close serial");
    } catch (boost::wrapexcept<boost::system::system_error> & e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), __FUNCTION__ << ": " << e.what());
      std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }
  } else {
    RCLCPP_WARN_STREAM(this->get_logger(), "already closed");
  }
}

void RoboteqSerialInterface::readSerial()
{
  using boost::asio::buffers_begin;
  using boost::asio::buffers_end;
  using boost::asio::read;
  using boost::asio::streambuf;
  using boost::asio::transfer_at_least;

  std::string buf_str;

  while (rclcpp::ok()) {
    if (serial_.is_open()) {
      try {
        streambuf response;
        // 6つの情報で最大32文字、バッファを取って48文字とする
        read(serial_, response, transfer_at_least(48));
        buf_str += std::string(buffers_begin(response.data()), buffers_end(response.data()));
        // convert "\n\n" or "\r\r" or "\n" or "\r" to ',' in order to make it easy to split
        buf_str = std::regex_replace(buf_str, std::regex("\n\n|\r\r|\n|\r"), ",");
        std::vector<std::string> ary = split(buf_str, ',');

        for (uint32_t i = 0; i < ary.size(); i++) {
          // 一番最後はデータが欠けている可能性があるため、その手前までを処理
          if (i == (ary.size() - 1)) break;

          const auto & e = ary.at(i);
          // string size is 0 and includes '='
          if (!e.empty() && e.find("=") != std::string::npos) receiveCallback(e);

          // delete already processed command
          buf_str.erase(0, e.size() + 1);
        }
      } catch (boost::wrapexcept<boost::system::system_error> & e) {
        RCLCPP_ERROR_STREAM(this->get_logger(), __FUNCTION__ << ": " << e.what());
        closeSerial();
      }
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(), "reopen serial");
      openSerial();
    }

    std::this_thread::sleep_for(std::chrono::microseconds(10000));
  }
}

void RoboteqSerialInterface::receiveCallback(const std::string & msg)
{
  std::vector<std::string> ary = split(msg, '=');
  if (ary.size() < 2 || ary.at(1).empty()) {
    return;
  }

  // Read Absolute Brushless Counter
  if (ary[0] == "CB") {
    onCB(ary.at(1));
  } else if (ary[0] == "BS") {  // Read Encoder Motor Speed in RPM
    onBS(ary.at(1));
  } else if (ary[0] == "FF") {  // read fault flags
    onFF(ary.at(1));
  } else if (ary[0] == "V") {  // read volts
    onV(ary.at(1));
  } else if (ary[0] == "BA") {  // read battery amps
    onBA(ary.at(1));
  } else if (ary[0] == "D") {  // read digital inputs
    onD(ary.at(1));
  } else if (ary[0] == "DO") {  // read digital output status
    onDO(ary.at(1));
  } else if (ary[0] == "AI") {  // read analog inputs
    onAI(ary.at(1));
  } else {
    RCLCPP_WARN_STREAM_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "Unknown Words: " << ary.at(0) << "," << ary.at(1));
  }

  publishRoboteqStatus();
}

void RoboteqSerialInterface::publishRoboteqStatus()
{
  if (
    battery_volts_ptr_ && fault_flag_ptr_ && rpm_ptr_ && analog_input_1_ptr_ &&
    digital_input_ptr_ && digital_output_ptr_) {
    roboteq_msgs::msg::RoboteqStatusStamped status;
    status.stamp = this->now();
    status.status.battery.battery_volts = *battery_volts_ptr_;
    status.status.fault.status = *fault_flag_ptr_;
    status.status.motor_speed_rpm = *rpm_ptr_;
    status.status.analog_input_1 = *analog_input_1_ptr_;
    status.status.digital_input = *digital_input_ptr_;
    status.status.digital_output = *digital_output_ptr_;
    status_pub_->publish(status);

    // reset
    battery_volts_ptr_.reset();
    fault_flag_ptr_.reset();
    rpm_ptr_.reset();
    analog_input_1_ptr_.reset();
    digital_input_ptr_.reset();
    digital_output_ptr_.reset();

    RCLCPP_DEBUG(this->get_logger(), "Status is published, reset");
  } else {
    RCLCPP_DEBUG(this->get_logger(), "topics are not satisfied for Status");
  }
}

void RoboteqSerialInterface::onCB(const std::string & msg)
{
  try {
    // Add processing if needed
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "CB " << e.what() << ", " << msg);
  }
}

void RoboteqSerialInterface::onBS(const std::string & msg)
{
  try {
    rpm_ptr_ = std::make_unique<int32_t>(std::stoi(msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "BS " << e.what() << ", " << msg);
    rpm_ptr_.reset();
  }
}

void RoboteqSerialInterface::onFF(const std::string & msg)
{
  try {
    fault_flag_ptr_ = std::make_unique<FaultStatus::_status_type>(std::stoi(msg));
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "FF " << e.what() << ", " << msg);
    fault_flag_ptr_.reset();
  }
}

void RoboteqSerialInterface::onV(const std::string & msg)
{
  try {
    battery_volts_ptr_ = std::make_unique<float>(std::stod(msg) * 0.10);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "V " << e.what() << ", " << msg);
    battery_volts_ptr_.reset();
  }
}

void RoboteqSerialInterface::onBA(const std::string & msg)
{
  try {
    // Add processing if needed
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "BA " << e.what() << ", " << msg);
  }
}

void RoboteqSerialInterface::onD(const std::string & msg)
{
  try {
    digital_input_ptr_ = std::make_unique<RoboteqStatus::_digital_input_type>();
    for (std::size_t i = 0; i < digital_input_ptr_->size(); i++) {
      digital_input_ptr_->at(i) = (std::stoi(msg) >> i) & 1;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "D " << e.what() << ", " << msg);
    digital_input_ptr_.reset();
  }
}

void RoboteqSerialInterface::onDO(const std::string & msg)
{
  try {
    digital_output_ptr_ = std::make_unique<RoboteqStatus::_digital_output_type>();
    for (std::size_t i = 0; i < digital_output_ptr_->size(); i++) {
      digital_output_ptr_->at(i) = (std::stoi(msg) >> i) & 1;
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "DO " << e.what() << ", " << msg);
    digital_output_ptr_.reset();
  }
}

void RoboteqSerialInterface::onAI(const std::string & msg)
{
  try {
    analog_input_1_ptr_ = std::make_unique<uint64_t>(std::stod(msg) * 0.001);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "AI " << e.what() << ", " << msg);
    analog_input_1_ptr_.reset();
  }
}

std::vector<std::string> RoboteqSerialInterface::split(const std::string & input, char delimiter)
{
  std::istringstream stream(input);

  std::string field;
  std::vector<std::string> result;
  while (std::getline(stream, field, delimiter)) {
    result.push_back(field);
  }
  return result;
}

void RoboteqSerialInterface::onRoboteqCommand(
  const roboteq_msgs::msg::RoboteqCommandStamped::ConstSharedPtr msg)
{
  std::string s = "!S " + std::to_string(msg->command.speed) + "\r\r";
  if (serial_.is_open()) boost::asio::write(serial_, boost::asio::buffer(s.c_str(), s.size()));

  onDOutCommand(msg->command.digital_output);
}

void RoboteqSerialInterface::onDOutCommand(const roboteq_msgs::msg::DigitalOutCommand & msg)
{
  for (uint32_t i = 0; i < msg.data.size(); i++)  // i=0: brake, i=1: blinker
  {
    std::string s;
    switch (msg.data.at(i)) {
      case roboteq_msgs::msg::DigitalOutCommand::NONE:
        break;
      case roboteq_msgs::msg::DigitalOutCommand::HIGH:
        s = "!D1 " + std::to_string(i + 1) + "\r\r";
        break;
      case roboteq_msgs::msg::DigitalOutCommand::LOW:
        s = "!D0 " + std::to_string(i + 1) + "\r\r";
        break;
      default:
        RCLCPP_ERROR_THROTTLE(
          this->get_logger(), *get_clock(), 3000, "Value error: d_out_command = %d",
          msg.data.at(i));
        break;
    }

    if (s != "") {
      if (serial_.is_open()) boost::asio::write(serial_, boost::asio::buffer(s.c_str(), s.size()));
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RoboteqSerialInterface)
