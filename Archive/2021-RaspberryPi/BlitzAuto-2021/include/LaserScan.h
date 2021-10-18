#pragma once

#include <vector>
#include <stdlib.h>
#include <iostream>
#include "LaserPoint.h"

namespace Blitz {
    class LaserScan {
        public:
            std::vector<LaserPoint> points = std::vector<LaserPoint>();

            float GetAngle(float angle);
            std::vector<float> GetData();
    };
}