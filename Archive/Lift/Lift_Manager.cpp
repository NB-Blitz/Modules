#include "WPILib.h"
#include "Lift_Manager.hpp"
#include "ctre/Phoenix.h"

FRC::Lift_Manager::Lift_Manager() :
	Lift_Motor(0)
{

}

void FRC::Lift_Manager::moveLiftTo(double stickY)
{
	// Calculate Variables
	double height = Lift_Motor.GetSelectedSensorPosition(0);
	positionRef = ((stickY + 1.0) / 2) * MAX_LIFT_POS;
	positionError = positionRef - height;

	// Calculate Speed_Ref
	if (abs(positionError) < WINDOW)
	{
		speedRef = 0;
	}
	else if (positionError > 0)
	{
		speedRef = positionError * UP_GAIN;
		if (speedRef < UPPER_SPEED_MIN)
		{
			speedRef = UPPER_SPEED_MIN;
		}
	}
	else
	{
		speedRef = positionError * DOWN_GAIN;
		if (speedRef > LOWER_SPEED_MIN)
		{
			speedRef = LOWER_SPEED_MIN;
		}
	}

	// Limit Speed_Ref
	// (I understand Talons automatically do this but it is good practice)
	if (speedRef > 1)
	{
		speedRef = 1;
	}
	else if (speedRef < -1)
	{
		speedRef = -1;
	}
	else
	{
		// Ignore
	}

	Lift_Motor.Set(speedRef);

}

void FRC::Lift_Manager::moveLift(double stickY)
{
	Lift_Motor.Set(stickY);
}

void FRC::Lift_Manager::resetLift()
{
	// Reset Lift in Test Mode
	// Not Finished (Since we need switches and whatnot)
}

void FRC::Lift_Manager::resetEnc()
{
	Lift_Motor.SetSelectedSensorPosition(0,0,0);
}

void FRC::Lift_Manager::updateSD()
{
	SmartDashboard::PutNumber("Encoder Height", Lift_Motor.GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber("Speed Ref", speedRef);
	SmartDashboard::PutNumber("Position Error", positionError);
	SmartDashboard::PutNumber("Position Ref", positionRef);
}
