/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#pragma once

#include <vector>
#include <iostream>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUDRATE B230400  

struct LidarPoint
{
    float angle;
    float distance;
};

class Lidar
{
    public:
        Lidar();
        ~Lidar();

        void readLidar();

        std::vector<LidarPoint> laser_scan;

    private:
        int serial_;

        void startScan(int serial);
        bool readByte(int serial, char & byte);
        bool readExact(int serial, uint8_t* buffer, size_t size);
};