#include "LaserScan.h"

float Blitz::LaserScan::GetAngle(float angle)
{
    float closestDistance = 0;
    float closestAngle = 360;
    for (LaserPoint point : points)
    {
        if (abs(point.angle - angle) < closestAngle)
        {
            closestDistance = point.distance;
            closestAngle = abs(point.angle - angle);
        }
    }

    return closestDistance;
}

std::vector<float> Blitz::LaserScan::GetData()
{
    std::vector<float> data;
    for (LaserPoint point : points)
    {
        data.push_back(point.distance);
    }
    return data;
}