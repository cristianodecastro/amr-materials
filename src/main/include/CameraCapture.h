#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <cscore_cv.h>
#include <opencv2/core/mat.hpp>
#include <networktables/NetworkTableEntry.h>
#include "RobotConfig.h"


// The USB camera is closed between photo requests. The worker repeats only
// the cached still for dashboard viewers, with no live camera feed.
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
  bool Capture(cv::Mat& image);
  cs::UsbCamera camera_;
  cs::CvSink sink_;
  cs::CvSource lastPhoto_;
  cv::Mat preview_; // accessed only by the worker after Start()
  nt::NetworkTableEntry captureButton_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_=false, pending_=false, busy_=false, cameraActive_=false;
  std::string status_="Camera not started", lastFile_, lastSize_;
  unsigned saved_=0;
};
