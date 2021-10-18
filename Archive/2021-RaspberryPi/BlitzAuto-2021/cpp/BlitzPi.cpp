#include "BlitzPi.h"

Blitz::BlitzPi::BlitzPi() :
	cv("BlitzCam")
{
	NT.SetDouble("H Low" , 80);
	NT.SetDouble("S Low" , 160);
	NT.SetDouble("V Low" , 170);
	NT.SetDouble("H High", 140);
	NT.SetDouble("S High", 200);
	NT.SetDouble("V High", 220);

	while (true)
	{
		currentMode = (AutoMode)NT.GetDouble("Autonomous Mode", AutoMode::PAUSED);

		if (currentMode == AutoMode::SEARCH)
		{
			// Network Tables (Input)
			int hLow  = NT.GetDouble("H Low" , 0);
			int sLow  = NT.GetDouble("S Low" , 0);
			int vLow  = NT.GetDouble("V Low" , 0);
			int hHigh = NT.GetDouble("H High", 255);
			int sHigh = NT.GetDouble("S High", 255);
			int vHigh = NT.GetDouble("V High", 255);

			// Turn Ints into Scalars
			cv::Scalar HSVLow (hLow,  sLow,  vLow);
			cv::Scalar HSVHigh(hHigh, sHigh, vHigh);

			// OpenCV
			Point ball = cv.FindClosestBall(HSVLow, HSVHigh);

			// Network Tables (Output)
			NT.SetDouble("Ball X", ball.x);
			NT.SetDouble("Ball Y", ball.y);
			
			if (ball.x == 5.148 && ball.y == 5.148)
				NT.SetBool("See Ball", false);
			else
				NT.SetBool("See Ball", true);
		}
		else if (currentMode != AutoMode::PAUSED)
		{
			// Get Scans
			LaserScan scan = RPLidar.GetScan();

			// Calculate Position
			GeometryMsgs::Pose pose = RPLidar.GetPose(scan);
			cv.Draw(scan, pose);

			// Network Tables
			NT.SetDouble("Robot X", pose.position.x * 1000000);
			NT.SetDouble("Robot Y", pose.position.y * 1000000);
			NT.SetBool("Robot Change", true);
		}
	}
	RPLidar.Dispose();
}

int main(int argc, const char * argv[]) {
	Blitz::BlitzPi pi;
}