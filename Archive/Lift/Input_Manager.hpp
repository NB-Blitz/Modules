#ifndef SRC_INPUT_MANAGER_HPP_
#define SRC_INPUT_MANAGER_HPP_

#include "WPILib.h"

namespace FRC
{
	class Input_Manager
	{
	public:
		Input_Manager();

		// Objects
		Joystick stick;

		// Methods
		double getAxis(int id); // Gets Joystick Axis
		bool getButton(int id); // Gets Joystick Button
	};
}

#endif /* SRC_DRIVEMANAGER_HPP_ */
