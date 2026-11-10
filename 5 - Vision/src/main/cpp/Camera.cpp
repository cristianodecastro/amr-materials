#include "Camera.h"

void Camera::init(){
    cs::UsbCamera mainCamera = frc::CameraServer::GetInstance() -> StartAutomaticCapture();
    mainCamera.SetResolution(640, 480);

    frc::Shuffleboard::GetTab("MainData").Add("Camera", mainCamera).WithPosition(0, 0).WithSize(10, 5);
}

void Camera::TrackObject(){
    cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("ColorTrack", 640, 480);
    frc::Shuffleboard::GetTab("MainData").Add("CameraProcess", outputStream).WithPosition(0, 0).WithSize(10, 5);

    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    cv::Mat frame;

    bool break_loop = false;

    while (!break_loop) {
        if (cvSink.GrabFrame(frame) == 0) {
            outputStream.NotifyError(cvSink.GetError());
            continue;
        }

        cv::Mat hsv, mask, result;

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Scalar lower_blue(100, 100, 100);   
        cv::Scalar upper_blue(140, 255, 255);

        inRange(hsv, lower_blue, upper_blue, mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        int object_x = 0;
        int object_y = 0;

        double max_area = 0;
        int max_contour = 0;
        cv::Rect bounding_rect;

        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if ( area > max_area ) {
                max_area = area;
                max_contour = i;
                bounding_rect = cv::boundingRect(contours[i]);
                object_x = bounding_rect.x + (bounding_rect.width  / 2.0);
                object_y = bounding_rect.y + (bounding_rect.height / 2.0);
            }
        }

        frc::SmartDashboard::PutNumber("object_x", object_x );
        frc::SmartDashboard::PutNumber("object_y", object_y );

        if      ( object_x > (320 + 25 ) ){
            movement->InverseKinematics( 0, 20, 0 );

        }else if( object_y < (320 - 25 ) ){
            movement->InverseKinematics( 0, -20, 0 );

        }else{
            movement->InverseKinematics( 0, 0, 0 );
            break_loop = true;
        }

        if ( !hardware->GetStopButton() ){  // Stop the Motors when the Stop Button is pressed
            hardware->SetLeft ( 0 );
            hardware->SetBack ( 0 );
            hardware->SetRight( 0 );
        }else{
            hardware->SetLeft ( movement->desired_left_speed );
            hardware->SetBack ( movement->desired_back_speed );
            hardware->SetRight( movement->desired_right_speed );
        }


        cv::drawContours(frame, contours, static_cast<int>(max_contour), cv::Scalar(0, 255, 0), 5);

        outputStream.PutFrame(frame);
    }
}

std::string Camera::DetectObject(){

    cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo("ColorCheck", 640, 480);
    frc::Shuffleboard::GetTab("MainData").Add("CameraProcess", outputStream).WithPosition(0, 0).WithSize(10, 5);

    cs::CvSink cvSink = frc::CameraServer::GetInstance()->GetVideo();
    cv::Mat frame;

    while(true){
        if (cvSink.GrabFrame(frame) == 0) {
            outputStream.NotifyError(cvSink.GetError());
            continue;
        }else{
            break;
        }
    }

    cv::Mat hsv, mask, result;

    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

    /* BLUE */

    cv::Scalar lower_blue(100, 100, 100);   
    cv::Scalar upper_blue(140, 255, 255);
    inRange(hsv, lower_blue, upper_blue, mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours_b;
    cv::findContours(mask, contours_b, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double max_area_blue = 0;
    int max_contour_blue = 0;

    for (size_t i = 0; i < contours_b.size(); i++) {
        double area = cv::contourArea(contours_b[i]);
        if ( area > max_area_blue ) {
            max_area_blue = area;
            max_contour_blue = i;
        }
    }

    /* BLUE */


    /* RED */

    cv::Scalar lower_red( 0, 100, 100);   
    cv::Scalar upper_red(15, 255, 255);
    inRange(hsv, lower_red, upper_red, mask);

    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours_r;
    cv::findContours(mask, contours_r, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double max_area_red = 0;
    int max_contour_red = 0;

    for (size_t i = 0; i < contours_r.size(); i++) {
        double area = cv::contourArea(contours_r[i]);
        if ( area > max_area_red ) {
            max_area_red = area;
            max_contour_red = i;
        }
    }

    /*  RED  */

    /* GREEN */

    cv::Scalar lower_green(45, 25, 25);   
    cv::Scalar upper_green(75, 255, 255);
    inRange(hsv, lower_green, upper_green, mask);

    // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours_g;
    cv::findContours(mask, contours_g, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double max_area_green = 0;
    int max_contour_green = 0;

    for (size_t i = 0; i < contours_g.size(); i++) {
        double area = cv::contourArea(contours_g[i]);
        if ( area > max_area_green ) {
            max_area_green = area;
            max_contour_green = i;
        }
    }

    /* GREEN */

    if      ( max_area_blue > max_area_red && max_area_blue > max_area_green ){
        cv::drawContours(frame, contours_b, static_cast<int>(max_contour_blue), cv::Scalar(0, 255, 0), 5);
        outputStream.PutFrame(frame);
        return "blue";
    }else if( max_area_red > max_area_blue && max_area_red > max_area_green ){
        cv::drawContours(frame, contours_r, static_cast<int>(max_contour_red), cv::Scalar(0, 255, 0), 5);
        outputStream.PutFrame(frame);
        return "red";
    }else if( max_area_green > max_area_blue && max_area_green > max_area_red ){
        cv::drawContours(frame, contours_g, static_cast<int>(max_contour_green), cv::Scalar(0, 255, 0), 5);
        outputStream.PutFrame(frame);
        return "green";
    }else{
        return "No Object";
    }


}
