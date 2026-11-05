#include "CameraCapture.h"

#include <cerrno>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <cameraserver/CameraServer.h>
#include <frc/RobotState.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/ComplexWidget.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {
std::string RgbText(RgbColour rgb) {
  std::ostringstream text;
  text<<std::fixed<<std::setprecision(1)<<rgb.red<<", "<<rgb.green<<", "<<rgb.blue;
  return text.str();
}
void EnsureDirectory(const std::string& path) {
  if (::mkdir(path.c_str(),0755)!=0 && errno!=EEXIST)
    throw std::runtime_error("Cannot create "+path+": "+std::strerror(errno));
  struct stat info {};
  if (::stat(path.c_str(),&info)!=0 || !S_ISDIR(info.st_mode))
    throw std::runtime_error("Not a directory: "+path);
}
}

CameraCapture::~CameraCapture() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_=true;
  }
  wake_.notify_one();
  if (worker_.joinable()) worker_.join();
}

void CameraCapture::InitializeColourCalibration() {
  struct stat info {};
  if (::stat(camera_config::COLOUR_CALIBRATION_FILE,&info)==0) {
    std::string error;
    if (LoadColourCalibration(camera_config::COLOUR_CALIBRATION_FILE,calibration_,error))
      calibrationStatus_="Loaded saved colour references and ROI";
    else calibrationStatus_="Saved calibration rejected; using supplied-photo defaults: "+error;
  }
  auto& tab=frc::Shuffleboard::GetTab("Colour calibration");
  roiX_=tab.Add("ROI X",double(calibration_.roi.x)).WithPosition(0,0).WithSize(3,1).GetEntry();
  roiY_=tab.Add("ROI Y",double(calibration_.roi.y)).WithPosition(3,0).WithSize(3,1).GetEntry();
  roiWidth_=tab.Add("ROI width",double(calibration_.roi.width)).WithPosition(6,0).WithSize(3,1).GetEntry();
  roiHeight_=tab.Add("ROI height",double(calibration_.roi.height)).WithPosition(9,0).WithSize(3,1).GetEntry();
  roiX_.SetDouble(calibration_.roi.x); roiY_.SetDouble(calibration_.roi.y);
  roiWidth_.SetDouble(calibration_.roi.width); roiHeight_.SetDouble(calibration_.roi.height);
  applyRoi_=tab.Add("Apply ROI",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(0,1).WithSize(3,1).GetEntry();
  referenceSlot_.SetDefaultOption("Slot 1",0);
  for (int i=1;i<camera_config::COLOUR_REFERENCE_COUNT;++i)
    referenceSlot_.AddOption("Slot "+std::to_string(i+1),i);
  tab.Add("Reference slot",referenceSlot_).WithWidget(frc::BuiltInWidgets::kComboBoxChooser)
      .WithPosition(3,1).WithSize(3,1);
  referenceName_=tab.Add("Reference name","Surprise 1").WithPosition(6,1).WithSize(3,1).GetEntry();
  referenceName_.SetString("Surprise 1");
  captureReference_=tab.Add("Capture reference",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(9,1).WithSize(3,1).GetEntry();
  disableReference_=tab.Add("Disable selected reference",false).WithWidget(frc::BuiltInWidgets::kToggleButton)
      .WithPosition(0,2).WithSize(3,1).GetEntry();
  applyRoi_.SetBoolean(false); captureReference_.SetBoolean(false); disableReference_.SetBoolean(false);
  tab.AddString("Calibration status",[this] {
    std::lock_guard<std::mutex> lock(mutex_); return calibrationStatus_;
  }).WithPosition(3,2).WithSize(9,1);
  tab.AddString("References (RGB)",[this] {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string text;
    for (unsigned i=0;i<calibration_.references.size();++i) {
      const auto& ref=calibration_.references[i];
      text+=std::to_string(i+1)+": "+ref.name+(ref.enabled?" ON ":" OFF ")+RgbText(ref.mean)+"\n";
    }
    return text;
  }).WithPosition(0,3).WithSize(8,2);
  tab.AddString("Last mean RGB",[this] {
    std::lock_guard<std::mutex> lock(mutex_); return RgbText(colourResult_.estimate.mean);
  }).WithPosition(8,3).WithSize(4,1);
  tab.AddNumber("Crop max standard deviation",[this] {
    std::lock_guard<std::mutex> lock(mutex_); return colourResult_.estimate.maxStdDev;
  }).WithPosition(8,4).WithSize(4,1);
}

void CameraCapture::UpdateColourCalibration() {
  const bool apply=applyRoi_.GetBoolean(false),capture=captureReference_.GetBoolean(false),
      disable=disableReference_.GetBoolean(false);
  if (apply) applyRoi_.SetBoolean(false);
  if (capture) captureReference_.SetBoolean(false);
  if (disable) disableReference_.SetBoolean(false);
  if (!apply && !capture && !disable) return;
  const int slot=referenceSlot_.GetSelected();
  const std::string name=referenceName_.GetString("");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (int(apply)+int(capture)+int(disable)>1) { calibrationStatus_="Press only one calibration action"; return; }
    if (!frc::RobotState::IsDisabled()) { calibrationStatus_="Calibration ignored - Disable the robot, then press again"; return; }
    if (busy_) { calibrationStatus_="Camera busy - wait, then press again"; return; }
    if (slot<0 || slot>=static_cast<int>(calibration_.references.size())) { calibrationStatus_="Invalid reference slot"; return; }
    auto candidate=calibration_;
    std::string error;
    if (apply) {
      const double values[4]={roiX_.GetDouble(-1),roiY_.GetDouble(-1),roiWidth_.GetDouble(-1),roiHeight_.GetDouble(-1)};
      for (double v:values) if (!std::isfinite(v) || v<0 || v>640 || std::floor(v)!=v) {
        calibrationStatus_="ROI requires whole pixels in a 640x480 reference image"; return;
      }
      const ImageRegion next={int(values[0]),int(values[1]),int(values[2]),int(values[3])};
      if (!ValidColourRegion(next)) { calibrationStatus_="ROI must fit inside 640x480 and have nonzero size"; return; }
      const auto old=candidate.roi;
      if (next.x==old.x && next.y==old.y && next.width==old.width && next.height==old.height) {
        calibrationStatus_="ROI unchanged; existing references preserved"; return;
      }
      candidate.roi=next;
      for (auto& r:candidate.references) r.enabled=false;
    } else if (disable) candidate.references[slot].enabled=false;
    else candidate.references[slot]={name,{0,0,0},true}; // validate the requested name before opening camera
    if (!ValidateColourCalibration(candidate,error)) { calibrationStatus_=error; return; }
    if (!capture) {
      try { EnsureDirectory("/home/pi/SkillsChallenge"); }
      catch (const std::exception& e) { calibrationStatus_=e.what(); return; }
      if (!SaveColourCalibration(camera_config::COLOUR_CALIBRATION_FILE,candidate,error)) { calibrationStatus_=error; return; }
      calibration_=std::move(candidate);
      calibrationStatus_=apply?"ROI saved; references disabled - recapture them using the new crop":
          "Selected reference disabled and saved";
      colourResult_=ColourResult{}; // measurements from old settings no longer apply
      return;
    }
    calibrationStatus_="Taking a fresh reference photo for "+name+"...";
  }
  if (!QueueCapture(true,slot,name)) {
    std::lock_guard<std::mutex> lock(mutex_);
    calibrationStatus_="Reference capture not started - camera busy/unavailable";
  }
}

void CameraCapture::Start() {
  if (worker_.joinable()) return;
  InitializeColourCalibration();
  auto& tab=frc::Shuffleboard::GetTab("Camera");
  captureButton_=tab.Add("Take photo",false)
      .WithWidget(frc::BuiltInWidgets::kToggleButton).WithPosition(0,0).WithSize(2,1).GetEntry();
  captureButton_.SetBoolean(false); // discard a request left by a previous program
  tab.AddString("Capture status",[this] {
    std::lock_guard<std::mutex> lock(mutex_); return status_;
  }).WithPosition(2,0).WithSize(8,1);
  tab.AddBoolean("Camera capturing",[this] {
    std::lock_guard<std::mutex> lock(mutex_); return cameraActive_;
  })
      .WithPosition(10,0).WithSize(2,1);
  tab.AddString("Last saved file",[this] {
    std::lock_guard<std::mutex> lock(mutex_); return lastFile_;
  }).WithPosition(0,1).WithSize(12,1);
  tab.AddNumber("Photos saved this run",[this] {
    std::lock_guard<std::mutex> lock(mutex_); return static_cast<double>(saved_);
  }).WithPosition(0,2).WithSize(3,1);
  tab.AddString("Saved image size",[this] {
    std::lock_guard<std::mutex> lock(mutex_); return lastSize_;
  }).WithPosition(3,2).WithSize(3,1);
  tab.AddString("Instructions",[] {
    return "Press Take photo once; allow up to 8 seconds. Preview holds the last saved photo; USB camera is off between requests.";
  }).WithPosition(6,2).WithSize(6,1);

  try {
    auto* server=frc::CameraServer::GetInstance();
    // Do not register an automatic raw-camera stream: a dashboard viewing that
    // stream could otherwise keep the USB camera reading frames indefinitely.
    camera_=cs::UsbCamera("Piece photo camera",camera_config::DEVICE);
    camera_.SetConnectionStrategy(cs::VideoSource::kConnectionForceClose);
    const bool preferredMode=camera_.SetVideoMode(cs::VideoMode::kMJPEG,
        camera_config::WIDTH,camera_config::HEIGHT,camera_config::FPS);
    sink_=cs::CvSink("Piece photo capture");
    sink_.SetEnabled(false);
    sink_.SetSource(camera_);
    lastPhoto_=server->PutVideo("Last captured photo",
        camera_config::WIDTH,camera_config::HEIGHT);
    lastPhoto_.SetFPS(camera_config::PREVIEW_FPS);
    tab.Add("Last captured photo",lastPhoto_)
        .WithWidget(frc::BuiltInWidgets::kCameraStream).WithPosition(0,3).WithSize(12,6);
    frc::Shuffleboard::GetTab("Simple tasks").Add("Last task photo",lastPhoto_)
        .WithWidget(frc::BuiltInWidgets::kCameraStream).WithPosition(0,5).WithSize(8,5);
    lastCrop_=server->PutVideo("Piece colour crop",400,200);
    lastCrop_.SetFPS(camera_config::PREVIEW_FPS);
    auto& colours=frc::Shuffleboard::GetTab("Colour calibration");
    colours.Add("Full photo and ROI",lastPhoto_).WithWidget(frc::BuiltInWidgets::kCameraStream)
        .WithPosition(0,5).WithSize(8,5);
    colours.Add("Crop used for RGB mean",lastCrop_).WithWidget(frc::BuiltInWidgets::kCameraStream)
        .WithPosition(8,5).WithSize(4,3);
    colours.AddString("How to calibrate",[] {
      return "Disable robot. Put a piece in the ROI; choose a slot, type its name, press Capture reference. "
          "It takes one new photo and saves the reference on the robot. Blue needs a sample. "
          "Changing ROI disables all references; capture them again. RGB is lighting-dependent: recalibrate under arena lighting.";
    }).WithPosition(8,8).WithSize(4,3);
    // Keep an actual image in the preview area before the first photograph.
    preview_=cv::Mat(camera_config::HEIGHT,camera_config::WIDTH,CV_8UC3,
        cv::Scalar(32,32,32));
    cv::putText(preview_,"Press Take photo",cv::Point(40,camera_config::HEIGHT/2),
        cv::FONT_HERSHEY_SIMPLEX,1.0,cv::Scalar(255,255,255),2);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status_=preferredMode?"Camera off - press Take photo":
          "Camera off - requested mode unavailable; capture will try camera default";
    }
    worker_=std::thread(&CameraCapture::Run,this);
  } catch (const std::exception& error) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_=std::string("Camera initialization failed: ")+error.what();
  }
}

void CameraCapture::Periodic() {
  UpdateColourCalibration();
  if (!captureButton_.GetBoolean(false)) return;
  captureButton_.SetBoolean(false);
  QueueCapture(false);
}
unsigned CameraCapture::RequestColourAnalysis() { return QueueCapture(true); }
CameraCapture::ColourResult CameraCapture::GetColourResult() {
  std::lock_guard<std::mutex> lock(mutex_); return colourResult_;
}
unsigned CameraCapture::QueueCapture(bool colour,int referenceSlot,const std::string& name) {
  unsigned request;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!worker_.joinable()) return 0;
    if (busy_) {
      status_="Photo already in progress - wait for Saved PNG or Capture failed";
      return 0; // one outstanding photograph; no unbounded queue
    }
    request=++requestSequence_;
    pendingJob_={colour?request:0,referenceSlot,name,calibration_};
    if (colour) { colourResult_=ColourResult{}; colourResult_.request=request; colourResult_.status="Capturing fresh colour photo..."; }
    busy_=pending_=true;
    status_="Opening camera for photo...";
  }
  wake_.notify_one();
  return request;
}

void CameraCapture::Run() {
  for (;;) {
    bool capture=false;
    CaptureJob job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      // Network camera widgets wait for new frames, including after reconnect.
      // Repeat the cached still at 1 fps; never reopen USB for preview updates.
      wake_.wait_for(lock,std::chrono::milliseconds(1000/camera_config::PREVIEW_FPS),
          [this] { return stopping_ || pending_; });
      if (stopping_) return;
      capture=pending_;
      if (capture) job=pendingJob_;
      pending_=false;
    }
    if (capture) {
      TakePhoto(job);
      std::lock_guard<std::mutex> lock(mutex_);
      busy_=false;
    }
    // Encoding/serving occurs only when someone views the still-photo source.
    if (lastPhoto_.IsEnabled() && !preview_.empty()) lastPhoto_.PutFrame(preview_);
    if (lastCrop_.IsEnabled() && !cropPreview_.empty()) lastCrop_.PutFrame(cropPreview_);
  }
}

void CameraCapture::CloseCamera() {
  sink_.SetEnabled(false);
  // Force-close wins even if an external CameraServer client requests video.
  camera_.SetConnectionStrategy(cs::VideoSource::kConnectionForceClose);
  std::lock_guard<std::mutex> lock(mutex_);
  cameraActive_=false;
}

void CameraCapture::TakePhoto(const CaptureJob& job) {
  const unsigned analysisRequest=job.analysis;
  try {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      cameraActive_=true;
    }
    camera_.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
    sink_.SetEnabled(true);
    const auto start=std::chrono::steady_clock::now();
    auto firstFrame=start;
    bool haveFrame=false,ready=false;
    std::string cameraError;
    cv::Mat image;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()<
           camera_config::REQUEST_TIMEOUT_S) {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (stopping_) break;
      }
      const auto timestamp=sink_.GrabFrame(image,0.25);
      if (timestamp==0 || image.empty()) {
        cameraError=sink_.GetError();
        continue;
      }
      const auto now=std::chrono::steady_clock::now();
      if (!haveFrame) { firstFrame=now; haveFrame=true; }
      // Brief warmup lets a freshly reopened camera adjust exposure. This loop
      // exists only during a button request, with a bounded timeout.
      if (std::chrono::duration<double>(now-firstFrame).count()>=camera_config::WARMUP_S) {
        ready=true; break;
      }
    }
    CloseCamera(); // USB capture is already stopped before rotation/PNG encoding
    if (!ready) throw std::runtime_error("No fresh photo from /dev/video"+
        std::to_string(camera_config::DEVICE)+" within "+
        std::to_string(camera_config::REQUEST_TIMEOUT_S)+" s"+
        (cameraError.empty()?"":": "+cameraError));
    cv::flip(image,image,-1);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      status_="Photo captured - USB camera off; saving PNG...";
    }
    const auto scaled=ScaleColourRegion(job.calibration.roi,image.cols,image.rows);
    if (scaled.width<=0 || scaled.height<=0) throw std::runtime_error("Colour ROI is invalid at this capture resolution");
    const cv::Rect roi(scaled.x,scaled.y,scaled.width,scaled.height);
    cv::Mat crop=image(roi).clone();
    cv::resize(crop,cropPreview_,cv::Size(400,200));
    if (analysisRequest) {
      cv::Scalar mean,deviation;
      cv::meanStdDev(crop,mean,deviation); // OpenCV channel order is B,G,R
      const RgbColour rgb={mean[2],mean[1],mean[0]};
      const double spread=std::max(deviation[0],std::max(deviation[1],deviation[2]));
      {
        std::lock_guard<std::mutex> lock(mutex_);
        colourResult_.estimate.mean=rgb; colourResult_.estimate.maxStdDev=spread;
      }
      auto settings=job.calibration;
      if (job.referenceSlot>=0) {
        if (!frc::RobotState::IsDisabled())
          throw std::runtime_error("Reference cancelled - keep robot Disabled until capture finishes");
        if (spread>camera_config::COLOUR_MAX_STDDEV)
          throw std::runtime_error("Reference crop is not uniform - keep floor, gripper and piece edges outside the ROI");
        settings.references.at(job.referenceSlot)={job.referenceName,rgb,true};
        EnsureDirectory("/home/pi/SkillsChallenge");
        std::string error;
        if (!SaveColourCalibration(camera_config::COLOUR_CALIBRATION_FILE,settings,error))
          throw std::runtime_error("Reference not saved: "+error);
        std::lock_guard<std::mutex> lock(mutex_);
        calibration_=settings;
        calibrationStatus_="Saved "+job.referenceName+" reference; mean RGB "+RgbText(rgb);
      }
      const auto estimate=ClassifyAverageColour(rgb,spread,settings.references.data(),
          static_cast<int>(settings.references.size()),camera_config::COLOUR_MAX_DISTANCE,
          camera_config::COLOUR_MIN_MARGIN,camera_config::COLOUR_MAX_STDDEV);
      const bool saved=Capture(image); // saved PNG keeps the unannotated image
      preview_=image.clone();
      cv::rectangle(preview_,roi,cv::Scalar(255,255,255),2);
      cv::putText(preview_,estimate.name+" RGB "+RgbText(rgb),cv::Point(20,35),
          cv::FONT_HERSHEY_SIMPLEX,0.8,cv::Scalar(255,255,255),2);
      std::lock_guard<std::mutex> lock(mutex_);
      colourResult_.request=analysisRequest; colourResult_.ready=colourResult_.success=true;
      colourResult_.estimate=estimate;
      colourResult_.status=estimate.reference<0?"Unknown - crop mixed, outside calibrated RGB range or ambiguous; calibrate if needed":
          "Detected "+estimate.name;
      colourResult_.status+="; mean RGB "+RgbText(rgb);
      if (!saved) colourResult_.status+="; PNG was not saved (see Camera status)";
    } else if (Capture(image)) {
      preview_=image.clone();
      cv::rectangle(preview_,roi,cv::Scalar(255,255,255),2);
    }
  } catch (const std::exception& error) {
    CloseCamera();
    std::lock_guard<std::mutex> lock(mutex_);
    status_=std::string("Capture failed (camera off): ")+error.what();
    if (analysisRequest) {
      colourResult_.request=analysisRequest; colourResult_.ready=true; colourResult_.success=false;
      colourResult_.status=status_;
    }
    if (job.referenceSlot>=0) calibrationStatus_=status_;
  }
}

bool CameraCapture::Capture(cv::Mat& image) {
  std::string temporaryFile;
  try {
    EnsureDirectory("/home/pi/SkillsChallenge");
    EnsureDirectory(camera_config::CAPTURE_DIRECTORY);
    struct statvfs space {};
    if (::statvfs(camera_config::CAPTURE_DIRECTORY,&space)!=0)
      throw std::runtime_error("Cannot check capture disk space");
    if (static_cast<unsigned long long>(space.f_bavail)*space.f_frsize < 64ULL*1024*1024)
      throw std::runtime_error("Less than 64 MiB free; copy and remove old photos first");

    // VMX wall clocks can be unset. A process id, microsecond time and sequence
    // identify files without relying on a human-readable date being correct.
    const auto time=std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    unsigned sequence;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      sequence=saved_+1;
    }
    const std::string file=std::string(camera_config::CAPTURE_DIRECTORY)+"/piece-"+
        std::to_string(time)+"-"+std::to_string(::getpid())+"-"+std::to_string(sequence)+".png";
    temporaryFile=file+".partial.png";
    // Save the original color frame, without overlays or resizing. PNG adds no
    // further lossy compression to a possibly JPEG-compressed camera image.
    if (!cv::imwrite(temporaryFile,image)) throw std::runtime_error("PNG writer failed");
    if (::rename(temporaryFile.c_str(),file.c_str())!=0)
      throw std::runtime_error(std::string("Cannot finalize photo: ")+std::strerror(errno));
    std::lock_guard<std::mutex> lock(mutex_);
    ++saved_;
    lastFile_=file;
    lastSize_=std::to_string(image.cols)+" x "+std::to_string(image.rows);
    status_="Saved PNG - camera off; copy Last saved file from the robot";
    return true;
  } catch (const std::exception& error) {
    if (!temporaryFile.empty()) ::unlink(temporaryFile.c_str());
    std::lock_guard<std::mutex> lock(mutex_);
    status_=std::string("Capture failed: ")+error.what();
    return false;
  }
}
