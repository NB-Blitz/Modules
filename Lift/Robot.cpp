#include "WPILib.h"
#include "Input_Manager.hpp"
#include "Lift_Manager.hpp"

class Robot: public SampleRobot
{
	FRC::Input_Manager Input_Man; // Input Manager
	FRC::Lift_Manager Lift_Man;   // Lift Manager
	double joyY;				  // Joystick Y

public:
	Robot() :
		Input_Man(),
		Lift_Man()
	{
		joyY = 0;
	}

/*-----------------------------------------------------------------------------------------------
 * 	  _______   _              ____          __  __           _
 *	 |__   __| | |            / __ \        |  \/  |         | |
 *	    | | ___| | ___ ______| |  | |_ __   | \  / | ___   __| | ___
 *	    | |/ _ \ |/ _ \______| |  | | '_ \  | |\/| |/ _ \ / _` |/ _ \
 *	    | |  __/ |  __/      | |__| | |_) | | |  | | (_) | (_| |  __/
 *	    |_|\___|_|\___|       \____/| .__/  |_|  |_|\___/ \__,_|\___|
 *	                                | |
 *	                                |_|
 *----------------------------------------------------------------------------------------------*/
	void OperatorControl()
	{
		while (IsOperatorControl() && IsEnabled())
		{
			// Get Joystick Axis
			joyY = Input_Man.getAxis(1) * -1;

			// Set Position Based off Button Press
			if (Input_Man.getButton(6))
			{
				Lift_Man.moveLiftTo(-1);
			}
			else if (Input_Man.getButton(7))
			{
				Lift_Man.moveLiftTo(-0.6);
			}
			else if (Input_Man.getButton(8))
			{
				Lift_Man.moveLiftTo(-0.2);
			}
			else if (Input_Man.getButton(9))
			{
				Lift_Man.moveLiftTo(0.2);
			}
			else if (Input_Man.getButton(10))
			{
				Lift_Man.moveLiftTo(0.6);
			}
			else if (Input_Man.getButton(11))
			{
				Lift_Man.moveLiftTo(1);
			}
			else {
				Lift_Man.moveLift(joyY);
			}

			// Reset Encoder Button
			if (Input_Man.getButton(1))
			{
				Lift_Man.resetEnc();
			}

			// Update SmartDashboard
			Lift_Man.updateSD();


			Wait(0.005);
		}
	}

/*-----------------------------------------------------------------------------------------------
 *	  _______        _     __  __           _
 *	 |__   __|      | |   |  \/  |         | |
 *	    | | ___  ___| |_  | \  / | ___   __| | ___
 *	    | |/ _ \/ __| __| | |\/| |/ _ \ / _` |/ _ \
 *	    | |  __/\__ \ |_  | |  | | (_) | (_| |  __/
 *	    |_|\___||___/\__| |_|  |_|\___/ \__,_|\___|
 *
 *----------------------------------------------------------------------------------------------*/
	void Test()
	{
		while (IsTest() && IsEnabled())
		{
			// Calibrate Lift
			Lift_Man.resetLift();
		}
	}
};

START_ROBOT_CLASS(Robot)
