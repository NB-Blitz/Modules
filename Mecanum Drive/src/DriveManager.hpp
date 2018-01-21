#ifndef SRC_DRIVEMANAGER_HPP_
#define SRC_DRIVEMANAGER_HPP_

#include "WPILib.h"
#include "Arcade.hpp"
#include "Mecanum.hpp"

namespace FRC{

	class DriveManager
	{

	public:
		DriveManager();
		FRC::Arcade arcade;
		FRC::Mecanum mecanum;
		Joystick Driver_Joystick;
		Solenoid Drive_Changer_Left, Drive_Changer_Right;
		bool pnuematicsEngaged;
		void SwitchDrive();
		void drive(bool bypass);

	};

}




#endif
