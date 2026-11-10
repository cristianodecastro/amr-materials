/************************************
 * Author: Felipe Ferreira
 * Release version: 1.0.0.0
 * 
 * Modified by: 
 * Last modification date: 
 * New version:

*************************************/

#include "Robot.h"


int main() { 

    // Start MockDS
    Robot r;
    r.ds.Enable();

    Hardware hard;
    Movement move( &hard );
    Oms oms( &hard );

    Lidar lidar;

    delay(1000);


    while (true)
    {
        lidar.readLidar();

        for (const auto& p : lidar.laser_scan)
        {
                std::cout << "Angle: " << p.angle << " deg, Distance: " << p.distance << " mm" << std::endl;
        }
    }

    return 0; 
}
