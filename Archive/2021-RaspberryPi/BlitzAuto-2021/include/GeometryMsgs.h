#pragma once

namespace GeometryMsgs
{
    class Point
    {
        public:
            Point(float xp=0, float yp=0, float zp=0) {
                x = xp;
                y = yp;
                z = zp;
            };

            float x;
            float y;
            float z;
    };
    class Quaternion
    {
        public:
            Quaternion(float xp=0, float yp=0, float zp=0, float wp=0) {
                x = xp;
                y = yp;
                z = zp;
                w = wp;
            };

            float x;
            float y;
            float z;
            float w;
    };
    class Pose
    {
        public:
            Pose(Point p=Point(), Quaternion q=Quaternion()) {
                position = p;
                orientation = q;
            };

            Point position;
            Quaternion orientation;
    };
}