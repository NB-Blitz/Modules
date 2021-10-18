#include "BlitzPi.h"

using namespace cv;
using namespace std;

int main()
{
	Camera cam(0);
	GUIMgr gui;

	while (cam.cap.isOpened())
	{
		// Grab Frame
		Mat frame = cam.getFrame();
		if (frame.empty())
			break;
		
		// Threshold
		gui.threshold(frame);

		// Contours
		vector<vector<Point> > contours = gui.getContours(frame);

		// Drawing
		gui.clearDrawing(frame);
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
			rectangle(gui.drawing, Rect(avgX - 20, avgY - 20, 40, 40), Scalar(0, 100, 100), 2);

			// Position
			putText(gui.drawing, "(" + to_string(avgX) + "," + to_string(avgY) + ")", Point(avgX - 50, avgY - 40), FONT_HERSHEY_COMPLEX_SMALL, 1, Scalar(0, 100, 100));
		}
		rectangle(gui.drawing, Rect(300, 226, 500, 211), Scalar(0, 100, 100), 2);

		// Export
		imshow("Drawing", gui.drawing);
		imshow("Camera", frame);
		if (waitKey(30) >= 0)
			break;
	}
	return 0;
}

class Camera {
public:
	VideoCapture cap;

	Camera(int camID) {
		cap = VideoCapture(camID);
		cap.set(CAP_PROP_FRAME_HEIGHT, 160);
		cap.set(CAP_PROP_FRAME_WIDTH, 120);
	}

	Mat getFrame() {
		Mat frame;
		cap >> frame;
		return frame;
	}
};

class GUIMgr {
public:
	Mat drawing;

	GUIMgr() {
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
	}

	void threshold(Mat frame)
	{
		// Tune Window
		int rlow = getTrackbarPos("rlow", "Tune");
		int glow = getTrackbarPos("glow", "Tune");
		int blow = getTrackbarPos("blow", "Tune");
		int rhigh = getTrackbarPos("rhigh", "Tune");
		int ghigh = getTrackbarPos("ghigh", "Tune");
		int bhigh = getTrackbarPos("bhigh", "Tune");

		// Threshold
		inRange(frame, Scalar(rlow, glow, blow), Scalar(rhigh, ghigh, bhigh), frame);
	}

	vector<vector<Point> > getContours(Mat frame)
	{
		vector<Vec4i> hierarchy;
		vector<vector<Point> > contours;
		findContours(frame, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
		return contours;
	}

	void clearDrawing(Mat frame)
	{
		drawing = Mat::zeros(frame.size(), CV_8UC3);
	}
};