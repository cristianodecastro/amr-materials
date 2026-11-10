#pragma once

#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "Functions.h"
#include "Hardware.h"

#include <string>

#include <cmath>

class Oms
{
    public:
        Oms( Hardware * h ) : hardware{h}{}

        void oms_driver( double desired_height );
        void SetGripper( double angle );

        double height = 1000;

    private:

        Hardware * hardware;

};