#include <Arcade.hpp>
#include "WPILIB.h"
#include <cmath>

FRC::Arcade::Arcade() :
	Left_Front_M_Arcade(0),
	Left_Back_M_Arcade(1),
	Right_Front_M_Arcade(2),
	Right_Back_M_Arcade(3)
{

}


void FRC::Arcade::ArcadeDrive(Joystick joy)
{
	double turning = joy.GetTwist();
	double speed = joy.GetY();

	if((std::fabs(turning) < 0.05) && (std::fabs(speed) > 0.05))
	{//Standardly drives forwards or backwards.
		Left_Front_M_Arcade.Set(speed);
		Left_Back_M_Arcade.Set(speed);//These are negative, right? If they've been fixed, put them back to normal.
		Right_Front_M_Arcade.Set(-speed);
		Right_Back_M_Arcade.Set(-speed);
	}
	else if((std::fabs(turning) > 0.05) && (std::fabs(speed) < 0.05))
	{//Turns on the spot. Not necessary? I didn't think so
		Left_Front_M_Arcade.Set(-turning);
		Left_Back_M_Arcade.Set(-turning);//I'm running under the assumption that the back motors are
		Right_Front_M_Arcade.Set(-turning);
		Right_Back_M_Arcade.Set(-turning);
	}
	else if(turning > 0.05)
	{//We're just going to assume that left is -1, and right is 1.
		Left_Front_M_Arcade.Set(speed);
		Left_Back_M_Arcade.Set(-speed);
		Right_Front_M_Arcade.Set((speed-turning));
		Right_Back_M_Arcade.Set((-speed+turning));
	}
	else if(turning < -0.05)
	{//We're just going to assume that left is -1, and right is 1.
		Left_Front_M_Arcade.Set((speed-turning));
		Left_Back_M_Arcade.Set((-speed+turning));
		Right_Front_M_Arcade.Set(speed);
		Right_Back_M_Arcade.Set(-speed);
	}
	else
	{
		Left_Front_M_Arcade.Set(0);//Stops it when the joystick is neutral
		Left_Back_M_Arcade.Set(0);
		Right_Front_M_Arcade.Set(0);
		Right_Back_M_Arcade.Set(0);
	}
}

