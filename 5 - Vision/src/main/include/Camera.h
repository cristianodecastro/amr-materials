#pragma once

#include <frc/shuffleboard/Shuffleboard.h>
#include <cameraserver/CameraServer.h>
#include <opencv2/opencv.hpp>

#include "Movement.h"
#include "Hardware.h"

#include "string.h"


class Camera{
    public:
        Camera( Movement * m, Hardware * h ) : movement{m}, hardware{h}{ init(); };
        void TrackObject();
        std::string DetectObject();

        void init();

    
    private:
        Movement * movement;
        Hardware * hardware;
};