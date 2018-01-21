#include "DriveManager.hpp"
#include "WPILib.h"



FRC::DriveManager::DriveManager() :
	arcade(),
	mecanum(),
	Driver_Joystick(0),
	Drive_Changer_Left(0),
	Drive_Changer_Right(1)

{
	pnuematicsEngaged = false;
	Drive_Changer_Left.Set(false);
	Drive_Changer_Right.Set(false);
}

void FRC::DriveManager::SwitchDrive()
{
	if(pnuematicsEngaged)
	{
		Drive_Changer_Left.Set(false);
		Drive_Changer_Right.Set(false);
		pnuematicsEngaged = false;
	}
	else
	{
		Drive_Changer_Left.Set(true);
		Drive_Changer_Right.Set(true);
		pnuematicsEngaged = true;
	}
}

void FRC::DriveManager::drive(bool bypass)
{
	if(pnuematicsEngaged)
	{
		arcade.ArcadeDrive(Driver_Joystick);
	}
	else
	{
		mecanum.ramp(Driver_Joystick);
		mecanum.mecanumDrive(Driver_Joystick, bypass);
	}
}

/* 2 Solenoids
 * On = Arcade
 * Off = Mecanum
 *
 */
