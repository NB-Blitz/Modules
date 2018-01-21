/*
 * Arcade.hpp
 *
 *  Created on: Jan 20, 2018
 *      Author: KevenLeng
 */

#ifndef SRC_Arcade_HPP_
#define SRC_Arcade_HPP_

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"

namespace FRC{

	class Arcade
	{

	public:
		Arcade();
		WPI_TalonSRX Left_Front_M_Arcade, Left_Back_M_Arcade, Right_Front_M_Arcade, Right_Back_M_Arcade;

		void ArcadeDrive(Joystick joy);

	};

}


#endif /* SRC_Arcade_HPP_ */
