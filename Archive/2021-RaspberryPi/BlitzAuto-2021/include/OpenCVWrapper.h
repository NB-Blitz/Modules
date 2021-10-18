#pragma once

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>

#include "cameraserver/CameraServer.h"

#include "LaserScan.h"
#include "GeometryMsgs.h"

#define PI 3.14159265358979323846

using namespace cv;
using namespace std;

namespace Blitz
{
    class OpenCVWrapper {
        public:
            OpenCVWrapper(string streamName);
            
            void Draw(LaserScan& scan, GeometryMsgs::Pose pose);
            void Draw(Point2f ball);

            Point2f FindClosestBall(cv::Scalar HSVlow, cv::Scalar HSVhigh);
            void Upload(Mat frame);
            Mat GetCamFrame();

        private:
            cs::CvSource cvsource;
            cv::VideoCapture cam;

            const int WIDTH = 640;
            const int HEIGHT = 480;
            const double LASER_SCALE = 40.0;
            const double POS_SCALE = 300.0;
    };
}