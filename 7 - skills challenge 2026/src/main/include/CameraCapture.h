#pragma once

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

#include <cscore_cv.h>
#include <opencv2/core/mat.hpp>
#include <networktables/NetworkTableEntry.h>
#include <frc/smartdashboard/SendableChooser.h>
#include "RobotConfig.h"


// The USB camera is closed between photo requests. The worker repeats only
// the cached still for dashboard viewers, with no live camera feed.
class CameraCapture {
 public:
  struct ColourResult {
    unsigned request=0;
    bool ready=false,success=false;
    ColourEstimate estimate;
    std::string status="No colour photo taken";
  };
  CameraCapture() = default;
  ~CameraCapture();
  void Start();
  void Periodic();
  unsigned RequestColourAnalysis(); // zero if busy/unavailable; no request is queued
  ColourResult GetColourResult();

 private:
  void Run();
  struct CaptureJob {
    unsigned analysis=0;
    int referenceSlot=-1;
    std::string referenceName;
    ColourCalibration calibration;
  };
  unsigned QueueCapture(bool colour,int referenceSlot=-1,const std::string& name="");
  void TakePhoto(const CaptureJob& job);
  void InitializeColourCalibration();
  void UpdateColourCalibration();
  void CloseCamera();
  bool Capture(cv::Mat& image);
  cs::UsbCamera camera_;
  cs::CvSink sink_;
  cs::CvSource lastPhoto_;
  cs::CvSource lastCrop_;
  cv::Mat preview_; // accessed only by the worker after Start()
  cv::Mat cropPreview_;
  nt::NetworkTableEntry captureButton_;
  std::thread worker_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_=false, pending_=false, busy_=false, cameraActive_=false;
  std::string status_="Camera not started", lastFile_, lastSize_;
  unsigned saved_=0;
  unsigned requestSequence_=0;
  CaptureJob pendingJob_;
  ColourCalibration calibration_=DefaultColourCalibration();
  frc::SendableChooser<int> referenceSlot_;
  nt::NetworkTableEntry roiX_,roiY_,roiWidth_,roiHeight_,applyRoi_,referenceName_,captureReference_,disableReference_;
  nt::NetworkTableEntry colourCardName_[camera_config::COLOUR_CARD_COUNT];
  nt::NetworkTableEntry recalibrateColour_[camera_config::COLOUR_CARD_COUNT];
  std::string calibrationStatus_="Using measured photo references; Blue/spare slots need calibration";
  ColourResult colourResult_;
};
