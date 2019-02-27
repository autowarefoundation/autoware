
/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 */

#include <thread>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>

#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

#define __APP_NAME__ "autoware_spinnaker"

class SpinnakerCamera
{
public:
  SpinnakerCamera();
  ~SpinnakerCamera();
  void spin();
  void publish_image(int);

private:
  ros::NodeHandle nh_, private_nh_;
  image_transport::ImageTransport* image_transport_;
  std::vector<image_transport::Publisher> image_pubs_;
  Spinnaker::SystemPtr system_;
  Spinnaker::CameraList camList_;
  CameraPtr* pCamList_;

  // config
  int width_;
  int height_;
  double fps_;
  int dltl_;
  std::string format_;

  Spinnaker::GenApi::INodeMap* node_map_;
};

SpinnakerCamera::SpinnakerCamera()
  : nh_()
  , private_nh_("~")
  , system_(Spinnaker::System::GetInstance())
  , camList_(system_->GetCameras())
  , width_(0)
  , height_(0)
  , fps_(0)
  , dltl_(0)
{
  private_nh_.param("width", width_, 1440);
  private_nh_.param("height", height_, 1080);
  private_nh_.param("fps", fps_, 60.0);
  private_nh_.param("dltl", dltl_, 100000000);
  unsigned int num_cameras = camList_.GetSize();
  ROS_INFO_STREAM("[" << __APP_NAME__ << "] Number of cameras detected: " << num_cameras);
  if (num_cameras < 1)
  {
    ROS_ERROR("[%s] Error: This program requires at least 1 camera.", __APP_NAME__);
    camList_.Clear();
    system_->ReleaseInstance();
    std::exit(-1);
  }
  image_transport_ = new image_transport::ImageTransport(nh_);
  pCamList_ = new CameraPtr[num_cameras];

  try
  {
    vector<gcstring> strSerialNumbers(camList_.GetSize());
    for (int i = 0; i < camList_.GetSize(); i++)
    {
      pCamList_[i] = camList_.GetByIndex(i);
      pCamList_[i]->Init();
      node_map_ = &pCamList_[i]->GetNodeMap();
      // config
      pCamList_[i]->Width.SetValue(width_);
      pCamList_[i]->Height.SetValue(height_);
      CEnumerationPtr ptrDeviceType = pCamList_[i]->GetTLDeviceNodeMap().GetNode("DeviceType");

      if (IsAvailable(ptrDeviceType) && ptrDeviceType->GetCurrentEntry()->GetSymbolic() == "GEV")
      {
        ///////////////////////                 DeviceLinkThroughputLimit   /////////////////////////////
        CIntegerPtr ptrDeviceLinkThroughputLimit = node_map_->GetNode("DeviceLinkThroughputLimit");
        if (IsAvailable(ptrDeviceLinkThroughputLimit) && IsWritable(ptrDeviceLinkThroughputLimit))
        {
          ROS_INFO_STREAM("[" << __APP_NAME__
                              << "] DeviceLinkThroughputLimit: " << ptrDeviceLinkThroughputLimit->GetValue());
          ptrDeviceLinkThroughputLimit->SetValue(dltl_);
        }
        else
        {
          ROS_WARN("[%s] This camera does not support DeviceLinkThroughputLimit, using default value.", __APP_NAME__);
        }
      }

      ///////////////////////                 FrameRate                   /////////////////////////////
      CFloatPtr ptrAcquisitionFrameRate = node_map_->GetNode("AcquisitionFrameRate");
      CBooleanPtr ptrAcquisitionFrameRateEnable = node_map_->GetNode("AcquisitionFrameRateEnable");
      CEnumerationPtr ptrAcquisitionFrameRateAuto = pCamList_[i]->GetNodeMap().GetNode("AcquisitionFrameRateAuto");
      if (IsAvailable(ptrAcquisitionFrameRateAuto) && IsWritable(ptrAcquisitionFrameRateAuto))
      {
        CEnumEntryPtr ptrAcquisitionFrameRateAutoOff = ptrAcquisitionFrameRateAuto->GetEntryByName("Off");
        if (IsAvailable(ptrAcquisitionFrameRateAutoOff) && IsReadable(ptrAcquisitionFrameRateAutoOff))
        {
          int64_t FrameRateAutoOff = ptrAcquisitionFrameRateAutoOff->GetValue();
          ptrAcquisitionFrameRateAuto->SetIntValue(FrameRateAutoOff);
          ROS_INFO_STREAM("[" << __APP_NAME__ << "] Updated FrameRateAuto to Off");
        }
        else
        {
          ROS_INFO_STREAM("[" << __APP_NAME__ << "] Cannot update FrameRateAuto to Off");
        }
      }
      if (IsAvailable(ptrAcquisitionFrameRateEnable) && IsWritable(ptrAcquisitionFrameRateEnable))
      {
        ptrAcquisitionFrameRateEnable->SetValue(true);
      }
      if (IsAvailable(ptrAcquisitionFrameRate) && IsWritable(ptrAcquisitionFrameRate))
      {
        ptrAcquisitionFrameRate->SetValue(fps_);
        ROS_INFO_STREAM("[" << __APP_NAME__ << "] Set FrameRate to " << fps_);
      }
      else
      {
        ROS_WARN("[%s] This camera does not support AcquisitionFrameRate, using default value.", __APP_NAME__);
      }

      ///////////////////////                 PixelFormat                 /////////////////////////////
      CEnumerationPtr ptrPixelFormat = node_map_->GetNode("PixelFormat");
      if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat))
      {
        ROS_INFO_STREAM("[" << __APP_NAME__ << "] Current pixel Format"
                            << ptrPixelFormat->GetCurrentEntry()->GetSymbolic());
        /*gcstring pixel_format = format_.c_str();

        CEnumEntryPtr ptrPixelFormatSetup = ptrPixelFormat->GetEntryByName(pixel_format);
        if (IsAvailable(ptrPixelFormatSetup) && IsReadable(ptrPixelFormatSetup))
        {
          int64_t pixelFormatSetup = ptrPixelFormatSetup->GetValue();
          ptrPixelFormat->SetIntValue(ptrPixelFormatSetup);

          ROS_INFO_STREAM("[" << __APP_NAME__ << "] Pixel format set to " <<
                                            ptrPixelFormat->GetCurrentEntry()->GetSymbolic());
        }
        else
        {
          ROS_WARN("[%s] %s pixel format not available. Using default.", __APP_NAME__, format_.c_str());
        }*/
      }
      else
      {
        ROS_WARN("[%s] Pixel format cannot be changed on this camera. Using default.", __APP_NAME__);
      }

      ///////////////////////                 AcquisitionMode                 /////////////////////////////
      CEnumerationPtr ptrAcquisitionMode = pCamList_[i]->GetNodeMap().GetNode("AcquisitionMode");
      if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
      {
        ROS_FATAL("[%s] Unable to set acquisition mode to continuous (node retrieval; "
                  "camera ",
                  __APP_NAME__);
      }

      ///////////////////////                 ContinuousMode                  /////////////////////////////
      CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
      if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
      {
        ROS_FATAL("[%s] Unable to set acquisition mode to continuous (entry "
                  "'continuous' retrieval. Aborting...",
                  __APP_NAME__);
      }

      int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();
      ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

      ROS_INFO("[%s] [camera %d] acquisition mode set to continuous...", __APP_NAME__, i);

      strSerialNumbers[i] = "";
      CStringPtr ptrStringSerial = pCamList_[i]->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
      if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
      {
        strSerialNumbers[i] = ptrStringSerial->GetValue();
        ROS_INFO("[%s] [camera %d] serial number set to %s", __APP_NAME__, i, strSerialNumbers[i].c_str());
      }
      std::string topic("image_raw");
      if (camList_.GetSize() > 1)
        topic = "camera" + std::to_string(i) + "/" + topic;
      image_pubs_.push_back(image_transport_->advertise(topic, 100));
    }
  }
  catch (Spinnaker::Exception& e)
  {
    ROS_ERROR("[%s] Error: %s", __APP_NAME__, e.what());
  }
}

SpinnakerCamera::~SpinnakerCamera()
{
  for (int i = 0; i < camList_.GetSize(); i++)
  {
    pCamList_[i]->EndAcquisition();
    pCamList_[i]->DeInit();
    pCamList_[i] = NULL;
    ROS_INFO("[%s] Shutting down camera %d", __APP_NAME__, i);
  }
  delete[] pCamList_;
  node_map_ = NULL;
  camList_.Clear();
  system_->ReleaseInstance();
}

void SpinnakerCamera::publish_image(int index)
{
  while (ros::ok())
  {
    try
    {
      ImagePtr pResultImage = pCamList_[index]->GetNextImage();
      if (pResultImage->IsIncomplete())
      {
        ROS_WARN("[%s] [camera %d] Image incomplete with image status %d", __APP_NAME__, index,
                 pResultImage->GetImageStatus());
      }
      else
      {
        ros::Time receive_time = ros::Time::now();

        // create opencv mat
        int pixel_format = pResultImage->GetPixelFormat();
        std::string encoding_pattern;
        switch (pixel_format)
        {
          case PixelFormatEnums::PixelFormat_BayerRG8:
            encoding_pattern = "bayer_rggb8";
            break;
          case PixelFormatEnums::PixelFormat_BayerGR8:
            encoding_pattern = "bayer_grbg8";
            break;
          case PixelFormatEnums::PixelFormat_BayerGB8:
            encoding_pattern = "bayer_gbrg8";
            break;
          case PixelFormatEnums::PixelFormat_BayerBG8:
            encoding_pattern = "bayer_bggr8";
            break;
          case PixelFormatEnums::PixelFormat_RGB8:
            encoding_pattern = "rgb8";
            break;
          case PixelFormatEnums::PixelFormat_BGR8:
            encoding_pattern = "bgr8";
            break;
          default:
            encoding_pattern = "mono8";
        }
        unsigned int XPadding = pResultImage->GetXPadding();
        unsigned int YPadding = pResultImage->GetYPadding();
        unsigned int rowsize = pResultImage->GetWidth();
        unsigned int colsize = pResultImage->GetHeight();

        // create and publish ros message
        std_msgs::Header header;
        sensor_msgs::Image msg;
        header.stamp = receive_time;
        header.frame_id = (camList_.GetSize() > 1) ? "camera" + std::to_string(index) : "camera";

        msg.header = header;
        msg.height = pResultImage->GetHeight();
        msg.width = pResultImage->GetWidth();
        msg.encoding = encoding_pattern;
        msg.step = pResultImage->GetStride();

        size_t image_size = pResultImage->GetImageSize();
        msg.data.resize(image_size);
        memcpy(msg.data.data(), pResultImage->GetData(), image_size);

        image_pubs_[index].publish(msg);
      }
      pResultImage->Release();
    }
    catch (Spinnaker::Exception& e)
    {
      ROS_ERROR("[%s] Error: %s", __APP_NAME__, e.what());
    }
  }
}
void SpinnakerCamera::spin()
{
  for (int i = 0; i < camList_.GetSize(); i++)
  {
    pCamList_[i]->BeginAcquisition();
    ROS_INFO("[%s] [camera %d] started acquisition", __APP_NAME__, i);
  }
  vector<std::shared_ptr<std::thread>> threads(camList_.GetSize());
  for (int i = 0; i < camList_.GetSize(); i++)
  {
    threads[i] = make_shared<thread>(&SpinnakerCamera::publish_image, this, i);
  }
  for (auto& thread : threads)
  {
    thread->join();
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "spinnaker");
  SpinnakerCamera node;
  node.spin();
  return 0;
}
