/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_interface/string.h>
#include <string>

namespace socketcan_bridge
{
  TopicToSocketCAN::TopicToSocketCAN(ros::NodeHandle* nh, ros::NodeHandle* nh_param,
      can::DriverInterfaceSharedPtr driver)
    {
      can_topic_ = nh->subscribe<can_msgs::Frame>("sent_messages", 1000,
                    boost::bind(&TopicToSocketCAN::msgCallback, this, _1));
      driver_ = driver;
    };

  void TopicToSocketCAN::setup()
    {

      state_listener_ = driver_->createStateListener(
              can::StateInterface::StateDelegate(this, &TopicToSocketCAN::stateCallback));
    };

  void TopicToSocketCAN::msgCallback(const can_msgs::Frame::ConstPtr& msg)
    {
      // ROS_DEBUG("Message came from sent_messages topic");

      // translate it to the socketcan frame type.

      can_msgs::Frame m = *msg.get();  // ROS message
      can::Frame f;  // socketcan type

      // converts the can_msgs::Frame (ROS msg) to can::Frame (socketcan.h)
      convertMessageToSocketCAN(m, f);

      if (!f.isValid())  // check if the id and flags are appropriate.
      {
        // ROS_WARN("Refusing to send invalid frame: %s.", can::tostring(f, true).c_str());
        // can::tostring cannot be used for dlc > 8 frames. It causes an crash
        // due to usage of boost::array for the data array. The should always work.
        ROS_ERROR("Invalid frame from topic: id: %#04x, length: %d, is_extended: %d", m.id, m.dlc, m.is_extended);
        return;
      }
      bool res = driver_->send(f);
      if (!res)
      {
        ROS_ERROR("Failed to send message: %s.", can::tostring(f, true).c_str());
      }
    };



  void TopicToSocketCAN::stateCallback(const can::State & s)
    {
      std::string err;
      driver_->translateError(s.internal_error, err);
      if (!s.internal_error)
      {
        ROS_INFO("State: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
      }
      else
      {
        ROS_ERROR("Error: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
      }
    };
};  // namespace socketcan_bridge
