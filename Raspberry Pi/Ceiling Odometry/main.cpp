#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

int main()
{
	Mat frame;
	VideoCapture cap(0);
	cap.set(CAP_PROP_FRAME_HEIGHT, 160);
	cap.set(CAP_PROP_FRAME_WIDTH, 120);
	namedWindow("Camera");
	namedWindow("Drawing");
	namedWindow("Tune", WINDOW_AUTOSIZE);

	int rLowDef = 54;
	int gLowDef = 44;
	int bLowDef = 159;
	int rHighDef = 142;
	int gHighDef = 97;
	int bHighDef = 255;

	createTrackbar("rlow", "Tune", &rLowDef, 255);
	createTrackbar("glow", "Tune", &gLowDef, 255);
	createTrackbar("blow", "Tune", &bLowDef, 255);
	createTrackbar("rhigh", "Tune", &rHighDef, 255);
	createTrackbar("ghigh", "Tune", &gHighDef, 255);
	createTrackbar("bhigh", "Tune", &bHighDef, 255);

	while (cap.isOpened())
	{
		cap >> frame;
		if (frame.empty())
			break;

		// Tune Window
		int rlow = getTrackbarPos("rlow", "Tune");
		int glow = getTrackbarPos("glow", "Tune");
		int blow = getTrackbarPos("blow", "Tune");
		int rhigh = getTrackbarPos("rhigh", "Tune");
		int ghigh = getTrackbarPos("ghigh", "Tune");
		int bhigh = getTrackbarPos("bhigh", "Tune");

		// Threshold
		inRange(frame, Scalar( rlow, glow, blow ), Scalar( rhigh, ghigh, bhigh ), frame);

		vector<Vec4i> hierarchy;
		vector<vector<Point> > contours;
		Mat drawing = Mat::zeros(frame.size(), CV_8UC3);
		
		findContours(frame, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

		int avgX = 0;
		int avgY = 0;
		int count = 0;
		for (vector<Point> p : contours) {
			double area = contourArea(p);
			if (area < 100)
				continue;
			Moments M = moments(p);
			avgX += int(M.m10 / M.m00);
			avgY += int(M.m01 / M.m00);
			count++;
		}
		if (count != 0)
		{
			avgX /= count;
			avgY /= count;
			rectangle(drawing, Rect(avgX - 20, avgY - 20, 40, 40), Scalar(0, 100, 100), 2);
			
			//putText(drawing, to_string((int)timeSpan.count()) + " ms", Point(avgX - 40, avgY + 40), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 100, 100));
			
			// Position
			putText(drawing, "(" + to_string(avgX) + "," + to_string(avgY) + ")", Point(avgX - 50, avgY - 40), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 100, 100));
		}
		rectangle(drawing, Rect(300,226 , 500, 211), Scalar(0, 100, 100), 2);
		//circle(drawing, Point(285, 314), 192, Scalar(0, 100, 100));
		
		imshow("Drawing", drawing);
		imshow("Camera", frame);
		if (waitKey(30) >= 0)
			break;
	}
	return 0;
}
