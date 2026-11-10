/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#include "Lidar.h"

Lidar::Lidar()
{
    do{
        serial_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
        if (serial_ == -1)
            std::cout << "Serial open Failed" << std::endl;
        sleep(1);
    } while (serial_ < 0);

    termios tty{};
    if (tcgetattr(serial_, &tty) != 0)
    {
        std::cout << "Termios Get Failed" << std::endl;
        close(serial_);
    }

    cfsetospeed(&tty, BAUDRATE);
    cfsetispeed(&tty, BAUDRATE);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8-bit data
    tty.c_cflag |= CLOCAL | CREAD;               // Enable receiver
    tty.c_cflag &= ~PARENB;                      // No parity
    tty.c_cflag &= ~CSTOPB;                      // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                     // No hardware flow control

    tty.c_lflag = 0;                            
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;                         // 100 ms

    tcflush(serial_, TCIFLUSH);

    if (tcsetattr(serial_, TCSANOW, &tty) != 0)
    {
        std::cout << "Termios Set Failed" << std::endl;
        close(serial_);
    }

    startScan(serial_);

    laser_scan.resize(360);

    std::cout << "Lidar Started" << std::endl;

}

Lidar::~Lidar()
{
    close(serial_);
}

void Lidar::startScan(int serial)
{
    uint8_t start[] = {0xA5, 0x60}; // T-mini Plus Scan Command
    write(serial, start, sizeof(start));
    sleep(1);
}

void Lidar::readLidar()
{
    char b1, b2;

    // Packet Header - 0x55 AA
    while (true){
        if (!readByte(serial_, b1)) return;    // PH - LSB
        if (b1 != 0xAA) continue;

        if (!readByte(serial_, b2)) return;    // PH - MSB
        if (b2 == 0x55) break;
    }

    uint8_t header[8];

    if (!readExact(serial_, header, 8)) return;

    uint8_t CT  = header[0];    // Scanning frequency & package type
    uint8_t LSN = header[1];    // Sample quantity

    uint16_t FSA = header[2] | (header[3] << 8);    // Start angle
    uint16_t LSA = header[4] | (header[5] << 8);    // End angle
    uint16_t CS  = header[6] | (header[7] << 8);    // Check code

    if (LSN == 0) return;

    std::vector<uint8_t> sample_data(LSN * 3);      // T-mini: Si = 3 bytes

    if (!readExact(serial_, sample_data.data(), sample_data.size())) return;

    float start_angle = (FSA >> 1) / 64.0f;
    float end_angle   = (LSA >> 1) / 64.0f;

    float angle_diff = end_angle - start_angle;

    if (angle_diff < 0)
        angle_diff += 360.0f;


    for (int i = 0; i < LSN; i++)
    {
        /* T-mini Plus Distance Formula */
        uint8_t intensity = sample_data[i * 3 + 0]; // Si(1) Intensity
        uint8_t s2        = sample_data[i * 3 + 1]; // Si(2) Distance Low
        uint8_t s3        = sample_data[i * 3 + 2]; // Si(3) Distance High

        uint16_t distance = (static_cast<uint16_t>(s3) << 6) | (static_cast<uint16_t>(s2) >> 2);


        float angle = start_angle; 

        /* T-mini Plus Intermediate angle Formula */
        if (LSN > 1){
            angle = start_angle + angle_diff * i / (LSN - 1);
        }


        while (angle >= 360.0) angle -= 360.0;
        while (angle < 0.0)    angle += 360.0;

        if ( distance > 0 ){
            laser_scan[(int)angle].angle = angle;
            laser_scan[(int)angle].distance = distance;
        }
    } 

    // std::cout << "LSN: " << laser_scan.size() << std::endl;


    return;

}

bool Lidar::readByte(int serial, char & byte)
{
    return read(serial, &byte, 1) == 1;
}

bool Lidar::readExact(int serial, uint8_t* buffer, size_t size)
{
    size_t received = 0;

    while (received < size)
    {
        int n = read(serial, buffer + received, size - received);

        if (n > 0){ received += n; }
        else{ return false; }
    }

    return true;
}