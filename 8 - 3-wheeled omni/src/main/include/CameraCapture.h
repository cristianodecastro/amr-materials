#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <cscore_cv.h>
#include <opencv2/core/mat.hpp>
#include <networktables/NetworkTableEntry.h>
#include "RobotConfig.h"

// USB acquisition runs only on an explicit photo request. Preview updates
// repeat the cached still; camera work never blocks the robot control loop.
class CameraCapture {
 public:
  CameraCapture()=default;
  ~CameraCapture();
  void Start();
  void Periodic();
 private:
  void Run();
  void TakePhoto();
  void CloseCamera();
  bool Capture(const cv::Mat& image);
  cs::UsbCamera camera_;
  cs::CvSink sink_;
  cs::CvSource lastPhoto_;
  cv::Mat preview_; // worker-owned after Start()
  nt::NetworkTableEntry captureButton_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_=false,pending_=false,busy_=false,cameraActive_=false;
  std::string status_="Camera not started",lastFile_,lastSize_;
  unsigned saved_=0;
};
