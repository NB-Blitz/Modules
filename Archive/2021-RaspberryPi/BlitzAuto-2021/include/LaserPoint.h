#pragma once

namespace Blitz {
    class LaserPoint {
        public:
            LaserPoint(float a, float dist, int q) {
                angle = a;
                distance = dist;
                quality = q;
            };

            float angle = 0;
            float distance = 0;
            int quality = 0;
    };
}