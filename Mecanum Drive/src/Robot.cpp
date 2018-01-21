#include "DriveManager.hpp"
#include "WPILib.h"

class Robot: public SampleRobot
{
	FRC::DriveManager driveMan;

public:
	Robot() :
		driveMan()

		{

		}

	void OperatorControl()
	{
		while (IsOperatorControl() && IsEnabled())
		{
			driveMan.drive(true);
		}
	}
};

START_ROBOT_CLASS(Robot)
