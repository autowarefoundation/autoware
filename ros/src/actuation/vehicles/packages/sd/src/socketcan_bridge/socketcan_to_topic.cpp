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

#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_interface/string.h>
#include <can_msgs/Frame.h>
#include <string>

namespace can {
template<> can::FrameFilterSharedPtr tofilter(const XmlRpc::XmlRpcValue  &ct) {
  XmlRpc::XmlRpcValue t(ct);
  try{ // to read as integer
      uint32_t id = static_cast<int>(t);
      return tofilter(id);
  }
  catch(...){ // read as string
      return  tofilter(static_cast<std::string>(t));
  }
}
}

namespace socketcan_bridge
{
  SocketCANToTopic::SocketCANToTopic(ros::NodeHandle* nh, ros::NodeHandle* nh_param,
      can::DriverInterfaceSharedPtr driver)
    {
      can_topic_ = nh->advertise<can_msgs::Frame>("received_messages", 100);
        driver_ = driver;
    };

  void SocketCANToTopic::setup()
    {
      // register handler for frames and state changes.
      frame_listener_ = driver_->createMsgListener(
              can::CommInterface::FrameDelegate(this, &SocketCANToTopic::frameCallback));

      state_listener_ = driver_->createStateListener(
              can::StateInterface::StateDelegate(this, &SocketCANToTopic::stateCallback));
    };

  void SocketCANToTopic::setup(const can::FilteredFrameListener::FilterVector &filters){
    frame_listener_.reset(new can::FilteredFrameListener(driver_,
                                                         can::CommInterface::FrameDelegate(this, &SocketCANToTopic::frameCallback),
                                                         filters));

    state_listener_ = driver_->createStateListener(
            can::StateInterface::StateDelegate(this, &SocketCANToTopic::stateCallback));
  }

  void SocketCANToTopic::setup(XmlRpc::XmlRpcValue filters) {
      setup(can::tofilters(filters));
  }
  void SocketCANToTopic::setup(ros::NodeHandle nh) {
       XmlRpc::XmlRpcValue filters;
       if(nh.getParam("can_ids", filters)) return setup(filters);
       return setup();
  }


  void SocketCANToTopic::frameCallback(const can::Frame& f)
    {
      // ROS_DEBUG("Message came in: %s", can::tostring(f, true).c_str());
      if (!f.isValid())
      {
        ROS_ERROR("Invalid frame from SocketCAN: id: %#04x, length: %d, is_extended: %d, is_error: %d, is_rtr: %d",
                  f.id, f.dlc, f.is_extended, f.is_error, f.is_rtr);
        return;
      }
      else
      {
        if (f.is_error)
        {
          // can::tostring cannot be used for dlc > 8 frames. It causes an crash
          // due to usage of boost::array for the data array. The should always work.
          ROS_WARN("Received frame is error: %s", can::tostring(f, true).c_str());
        }
      }

      can_msgs::Frame msg;
      // converts the can::Frame (socketcan.h) to can_msgs::Frame (ROS msg)
      convertSocketCANToMessage(f, msg);

      msg.header.frame_id = "";  // empty frame is the de-facto standard for no frame.
      msg.header.stamp = ros::Time::now();

      can_topic_.publish(msg);
    };


  void SocketCANToTopic::stateCallback(const can::State & s)
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
