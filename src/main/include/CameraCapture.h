#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <cscore_cv.h>
#include <networktables/NetworkTableEntry.h>
#include "RobotConfig.h"


// The worker sleeps and the USB camera is closed between photo requests.
// CameraServer publishes only the requested still photo, with no live feed.
class CameraCapture {
 public:
  CameraCapture() = default;
  ~CameraCapture();
  void Start();
  void Periodic();

 private:
  void Run();
  void TakePhoto();
  void CloseCamera();
  void Capture(cv::Mat& image);
  cs::UsbCamera camera_;
  cs::CvSink sink_;
  cs::CvSource lastPhoto_;
  nt::NetworkTableEntry captureButton_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_=false, pending_=false, busy_=false, cameraActive_=false;
  std::string status_="Camera not started", lastFile_, lastSize_;
  unsigned saved_=0;
};
