#include "WPILib.h"
#include "Lift_Manager.hpp"
#include "ctre/Phoenix.h"

FRC::Lift_Manager::Lift_Manager() :
	Lift_Motor(0)
{

}

void FRC::Lift_Manager::moveLiftTo(double joyPos)
{
	// Calculate Variables
	double height = Lift_Motor.GetSelectedSensorPosition(0);
	Position_Ref = ((joyPos + 1.0) / 2) * MAX_LIFT_POS;
	Position_Error = Position_Ref - height;

	// Calculate Speed_Ref
	if (abs(Position_Error) < WINDOW)
	{
		Speed_Ref = 0;
	}
	else if (Position_Error > 0)
	{
		Speed_Ref = Position_Error * UP_GAIN;
		if (Speed_Ref < UPPER_SPEED_MIN)
		{
			Speed_Ref = UPPER_SPEED_MIN;
		}
	}
	else
	{
		Speed_Ref = Position_Error * DOWN_GAIN;
		if (Speed_Ref > LOWER_SPEED_MIN)
		{
			Speed_Ref = LOWER_SPEED_MIN;
		}
	}

	// Limit Speed_Ref
	// (I understand Talons automatically do this but it is good practice)
	if (Speed_Ref > 1)
	{
		Speed_Ref = 1;
	}
	else if (Speed_Ref < -1)
	{
		Speed_Ref = -1;
	}
	else
	{
		// Ignore
	}

	Lift_Motor.Set(Speed_Ref);

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
	SmartDashboard::PutNumber("Speed Ref", Speed_Ref);
	SmartDashboard::PutNumber("Position Error", Position_Error);
	SmartDashboard::PutNumber("Position Ref", Position_Ref);
}
