#include "WPILib.h"
#include "ctre/Phoenix.h"

class Robot : public SampleRobot
{
	Joystick Stick;
	WPI_TalonSRX Left_Front, Left_Back, Right_Front, Right_Back;
	PowerDistributionPanel PDP;
	double speeds[4], maxMagnitude;

public:
	Robot() :
		Stick(0),
		Left_Front(1),
		Left_Back(2),
		Right_Front(3),
		Right_Back(4),
		PDP(0)

	{
		maxMagnitude = 0;
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
			double joyX = Stick.GetX();
			double joyY = Stick.GetY();
			double joyZ = Stick.GetZ();

			speeds[0] = joyX + joyY + joyZ;
			speeds[1] = -joyX + joyY + joyZ;
			speeds[2] = -joyX + joyY - joyZ;
			speeds[3] = joyX + joyY - joyZ;

			// Sets maxMagnitude to the highest speed out of the 4 motors
			for (int i = 0; i < 4; i++)
			{
				double speed = fabs(speeds[i]);
				if (maxMagnitude < speed)
				{
					maxMagnitude = speed;
				}
			}

			// If maxMagnitude is over 1, divide all 4 motors by maxMagnitude
			// Ensures that the motor speeds are set within -1 and 1
			if (maxMagnitude > 1)
			{
				for (int i = 0; i < 4; i++)
				{
					speeds[i] /= maxMagnitude;
				}
			}

			// Deadband
			for (int i = 0; i < 4; i++)
			{
				if (fabs(speeds[i]) < .2)
				{
					speeds[i] = 0;
				}
			}

			Left_Front.Set(speeds[0]);
			Left_Back.Set(speeds[1]);
			Right_Front.Set(speeds[2]);
			Right_Back.Set(speeds[3]);

			SmartDashboard::PutNumber("Joystick X", joyX);
			SmartDashboard::PutNumber("Joystick Y", joyY);
			SmartDashboard::PutNumber("Joystick Z", joyZ);

			SmartDashboard::PutNumber("LF Speed", Left_Front.GetSelectedSensorVelocity(0));
			SmartDashboard::PutNumber("LB Speed", Left_Back.GetSelectedSensorVelocity(0));
			SmartDashboard::PutNumber("RF Speed", Right_Front.GetSelectedSensorVelocity(0));
			SmartDashboard::PutNumber("RB Speed", Right_Back.GetSelectedSensorVelocity(0));

			SmartDashboard::PutNumber("LF PDP Current", PDP.GetCurrent(0));
			SmartDashboard::PutNumber("LB PDP Current", PDP.GetCurrent(1));
			SmartDashboard::PutNumber("RF PDP Current", PDP.GetCurrent(15));
			SmartDashboard::PutNumber("RB PDP Current", PDP.GetCurrent(14));

			SmartDashboard::PutNumber("LF Talon Current", PDP.GetCurrent(0));
			SmartDashboard::PutNumber("LB Talon Current", PDP.GetCurrent(1));
			SmartDashboard::PutNumber("RF Talon Current", PDP.GetCurrent(15));
			SmartDashboard::PutNumber("RB Talon Current", PDP.GetCurrent(14));

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
			bool button9 = Stick.GetRawButton(9);
			bool button10 = Stick.GetRawButton(10);
			bool button11 = Stick.GetRawButton(11);
			bool button12 = Stick.GetRawButton(12);

			if (button9)
			{
				Left_Front.Set(1);
			}
			if (button10)
			{
				Left_Back.Set(1);
			}
			if (button11)
			{
				Right_Front.Set(1);
			}
			if (button12)
			{
				Right_Back.Set(1);
			}
		}
	}
};

START_ROBOT_CLASS(Robot)
