#include "CameraCapture.h"

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <unistd.h>

#include <cameraserver/CameraServer.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/ComplexWidget.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace {
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

void CameraCapture::Start() {
  if (worker_.joinable()) return;
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
  if (!captureButton_.GetBoolean(false)) return;
  captureButton_.SetBoolean(false);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!worker_.joinable()) return;
    if (busy_) {
      status_="Photo already in progress - wait for Saved PNG or Capture failed";
      return; // one outstanding photograph; no unbounded queue
    }
    busy_=pending_=true;
    status_="Opening camera for photo...";
  }
  wake_.notify_one();
}

void CameraCapture::Run() {
  for (;;) {
    bool capture=false;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      // Network camera widgets wait for new frames, including after reconnect.
      // Repeat the cached still at 1 fps; never reopen USB for preview updates.
      wake_.wait_for(lock,std::chrono::milliseconds(1000/camera_config::PREVIEW_FPS),
          [this] { return stopping_ || pending_; });
      if (stopping_) return;
      capture=pending_;
      pending_=false;
    }
    if (capture) {
      TakePhoto();
      std::lock_guard<std::mutex> lock(mutex_);
      busy_=false;
    }
    // Encoding/serving occurs only when someone views the still-photo source.
    if (lastPhoto_.IsEnabled() && !preview_.empty()) lastPhoto_.PutFrame(preview_);
  }
}

void CameraCapture::CloseCamera() {
  sink_.SetEnabled(false);
  // Force-close wins even if an external CameraServer client requests video.
  camera_.SetConnectionStrategy(cs::VideoSource::kConnectionForceClose);
  std::lock_guard<std::mutex> lock(mutex_);
  cameraActive_=false;
}

void CameraCapture::TakePhoto() {
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
    // Replace the cached preview only after successfully saving this photo.
    if (Capture(image)) preview_=image;
  } catch (const std::exception& error) {
    CloseCamera();
    std::lock_guard<std::mutex> lock(mutex_);
    status_=std::string("Capture failed (camera off): ")+error.what();
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
