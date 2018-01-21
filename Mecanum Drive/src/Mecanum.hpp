
#ifndef SRC_MECANUM_HPP_
#define SRC_MECANUM_HPP_

#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "AHRS.h"

namespace FRC
{
	class Mecanum
	{
	public:
		Mecanum();
		WPI_TalonSRX Left_Front_Motor_Mecanum, Left_Back_Motor_Mecanum, Right_Front_Motor_Mecanum, Right_Back_Motor_Mecanum;
		AHRS ahrs;

		void PICorrection(int id);
		void ramp(Joystick joy);
		void mecanumDrive(Joystick joy, bool bypass);

		double joyX = 0;
		double joyY = 0;
		double joyRotate = 0;
		double joyXRaw = 0;
		double joyYRaw = 0;
		double joyRotateRaw = 0;
		double speeds[4];
		//double delta;
		double rotation = 0;
		int dir = 1;
		double maxMagnitude;

		//PIcorrection
		int EncoderFreq[4];
		double spdRef[4];
		double SPFeedBack[4];
		double error[4];
		double intOut[4];
		double propOut[4];
		double PIout[4];
		double PWMout[4];
		double preSpeed[4];
	};
}

#endif /* SRC_MECANUM_HPP_ */
