#pragma once

#include <chrono>
#include <opencv2/core/core.hpp>

#include "RPLidarWrapper.h"
#include "VMXPiWrapper.h"
#include "NetworkTablesWrapper.h"
#include "OpenCVWrapper.h"

using namespace std;

namespace Blitz {
    enum AutoMode {
        BARREL,
        SLALOM,
        BOUNCE,
        SEARCH,
        PAUSED
    };
    
    class BlitzPi
    {
        public:
            BlitzPi();

        private:
            RPLidarWrapper RPLidar;
            NetworkTablesWrapper NT;
            OpenCVWrapper cv;

            AutoMode currentMode;
    };
}