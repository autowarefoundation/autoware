
#include "SpinGenApi/SpinnakerGenApi.h"
#include "Spinnaker.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <mutex>
#include <opencv2/core/core.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sstream>
#include <std_msgs/Header.h>
#include <thread>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

std::mutex mtx;

class SpinnakerCamera {
public:
  SpinnakerCamera();
  ~SpinnakerCamera();
  void spin();
  void publisher(int);

private:
  ros::NodeHandle nh_, private_nh_;
  image_transport::ImageTransport *image_transport_;
  std::vector<image_transport::Publisher> image_pubs_;
  // image_transport::Publisher image_pub_;
  Spinnaker::SystemPtr system_;
  Spinnaker::CameraList camList_;
  CameraPtr *pCamList_;

  // config
  int width_;
  int height_;
  double fps_;
  int dltl_;

  Spinnaker::GenApi::INodeMap *node_map_;
};

SpinnakerCamera::SpinnakerCamera()
    : nh_(), private_nh_("~"), system_(Spinnaker::System::GetInstance()),
      camList_(system_->GetCameras()), width_(0), height_(0), fps_(0),
      dltl_(0) {
  private_nh_.param("width", width_, 1280);
  private_nh_.param("height", height_, 960);
  private_nh_.param("fps", fps_, 30.0);
  private_nh_.param("dltl", dltl_, 50000000);
  unsigned int num_cameras = camList_.GetSize();
  ROS_INFO_STREAM_ONCE(
      "[SpinnakerCamera]: Number of cameras detected: " << num_cameras);
  image_transport_ = new image_transport::ImageTransport(nh_);
  pCamList_ = new CameraPtr[camList_.GetSize()];
  // image_pub_ = image_transport_->advertise(topic, 100);
  try {
    vector<gcstring> strSerialNumbers(camList_.GetSize());
    for (int i = 0; i < camList_.GetSize(); i++) {
      pCamList_[i] = camList_.GetByIndex(i);
      pCamList_[i]->Init();
      node_map_ = &pCamList_[i]->GetNodeMap();
      // config
      pCamList_[i]->Width.SetValue(width_);
      pCamList_[i]->Height.SetValue(height_);
      CIntegerPtr ptrDeviceLinkThroughputLimit =
          node_map_->GetNode("DeviceLinkThroughputLimit");
      ptrDeviceLinkThroughputLimit->SetValue(dltl_);
      CFloatPtr ptrAcquisitionFrameRate =
          node_map_->GetNode("AcquisitionFrameRate");
      CBooleanPtr ptrAcquisitionFrameRateEnable =
          node_map_->GetNode("AcquisitionFrameRateEnable");
      ptrAcquisitionFrameRateEnable->SetValue(true);
      ptrAcquisitionFrameRate->SetValue(fps_);

      CEnumerationPtr ptrAcquisitionMode =
          pCamList_[i]->GetNodeMap().GetNode("AcquisitionMode");
      if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode)) {
        cout << "Unable to set acquisition mode to continuous (node retrieval; "
                "camera "
             << i << "). Aborting..." << endl
             << endl;
      }

      CEnumEntryPtr ptrAcquisitionModeContinuous =
          ptrAcquisitionMode->GetEntryByName("Continuous");
      if (!IsAvailable(ptrAcquisitionModeContinuous) ||
          !IsReadable(ptrAcquisitionModeContinuous)) {
        cout << "Unable to set acquisition mode to continuous (entry "
                "'continuous' retrieval "
             << i << "). Aborting..." << endl
             << endl;
      }

      int64_t acquisitionModeContinuous =
          ptrAcquisitionModeContinuous->GetValue();

      ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

      cout << "Camera " << i << " acquisition mode set to continuous..."
           << endl;

      strSerialNumbers[i] = "";
      CStringPtr ptrStringSerial =
          pCamList_[i]->GetTLDeviceNodeMap().GetNode("DeviceSerialNumber");
      if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial)) {
        strSerialNumbers[i] = ptrStringSerial->GetValue();
        cout << "Camera " << i << " serial number set to "
             << strSerialNumbers[i] << "..." << endl;
      }
      std::string topic(std::string("image_raw"));
      if (camList_.GetSize() > 1)
        topic = "camera" + std::to_string(i) + "/" + topic;
      image_pubs_.push_back(image_transport_->advertise(topic, 10));
    }
  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
  }
}

SpinnakerCamera::~SpinnakerCamera() {
  for (int i = 0; i < camList_.GetSize(); i++) {
    // pCam_ = camList_.GetByIndex(i);
    pCamList_[i]->EndAcquisition();
    pCamList_[i]->DeInit();
    pCamList_[i] = NULL;
  }
  delete[] pCamList_;
  node_map_ = NULL;
  camList_.Clear();
  system_->ReleaseInstance();
}

void SpinnakerCamera::publisher(int index) {
  while(ros::ok()){
  try {
    mtx.lock();
    ImagePtr pResultImage = pCamList_[index]->GetNextImage();
    if (pResultImage->IsIncomplete()) {
      cout << "[camera" << index << "] Image incomplete with image status "
           << pResultImage->GetImageStatus() << "..." << endl
           << endl;
    } else {
      ros::Time receive_time = ros::Time::now();

      // create opencv mat
      ImagePtr convertedImage =
          pResultImage->Convert(PixelFormat_BGR8, NEAREST_NEIGHBOR);
      unsigned int XPadding = convertedImage->GetXPadding();
      unsigned int YPadding = convertedImage->GetYPadding();
      unsigned int rowsize = convertedImage->GetWidth();
      unsigned int colsize = convertedImage->GetHeight();

      // image data contains padding. When allocating Mat container size,
      // you need to account for the X,Y image data padding.
      cv::Mat cvimg =
          cv::Mat(colsize + YPadding, rowsize + XPadding, CV_8UC3,
                  convertedImage->GetData(), convertedImage->GetStride());
      // create and publish ros message
      std_msgs::Header header;
      header.stamp = receive_time;
      header.frame_id = "camera";
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(header, "bgr8", cvimg).toImageMsg();
      image_pubs_[index].publish(msg);
    }
    pResultImage->Release();
    mtx.unlock();
  } catch (Spinnaker::Exception &e) {
    cout << "Error: " << e.what() << endl;
  }
  }
}
void SpinnakerCamera::spin() {
  for (int i = 0; i < camList_.GetSize(); i++) {
    pCamList_[i]->BeginAcquisition();
    cout << "Camera " << i << " started acquiring images..." << endl;
  }
  vector<std::shared_ptr<std::thread>> threads(camList_.GetSize());
  for (int i = 0; i < camList_.GetSize(); i++) {
    threads[i] = make_shared<thread>(&SpinnakerCamera::publisher, this, i);
  }
  for (auto &thread : threads) {
    thread->join();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "spinnaker");
  SpinnakerCamera node;
  node.spin();
  return 0;
}
