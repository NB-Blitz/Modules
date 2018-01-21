#include <Mecanum.hpp>
#include "WPILib.h"


FRC::Mecanum::Mecanum() :
	Left_Front_Motor_Mecanum(0),
	Left_Back_Motor_Mecanum(1),
	Right_Front_Motor_Mecanum(2),
	Right_Back_Motor_Mecanum(3),

	ahrs {SPI::Port::kMXP}

{

}

void FRC::Mecanum::PICorrection(int id)
{
	double const RateFrequency = 2000; //target Velocity
	double propGain = 1.0; //Proportional multiplier
	double const MaxHz = 2600.0; // max Hz

	if(speeds[id] - preSpeed[id] > .1)
	{
		// Original
		//propGain = 4;

		// WPT
		propGain = 1.5;
	}
	else
	{
		// Original
		//propGain = 1.5;

		// WPT
		propGain = 1;
	}

	spdRef[id] = speeds[id] * (RateFrequency/MaxHz);

	SPFeedBack[id] = EncoderFreq[id]/RateFrequency;

	error[id] = spdRef[id] - SPFeedBack[id];

	propOut[id] = error[id] * propGain;

	PIout[id] = propOut[id];

	PWMout[id] = spdRef[id] + PIout[id];

	preSpeed[id] = speeds[id];
}

void FRC::Mecanum::ramp(Joystick joy)
{
	double deadBand = 0.2;	//deadband range
	// Original
	//double rampRate = 1/(.05 * 200.0); //.5 = time in seconds, 200 = scan rate

	// WPT
	double rampRate = 1/(.1 * 200.0); //.5 = time in seconds, 200 = scan rate

	joyXRaw = dir * joy.GetX();
	joyYRaw = dir * -joy.GetY();
	joyRotateRaw = joy.GetRawAxis(2) * .75;

	//Left X axis
	if(fabs(joyXRaw) < deadBand)
	{
		joyXRaw = 0;
	}
	if(joyXRaw + .02 >= joyX && joyXRaw - .02 <= joyX)
	{
		//WAT???????????????
	}
	else if(joyXRaw > joyX)
	{
		joyX += rampRate;
	}
	else
	{
		joyX -= rampRate;
	}

	//Left Y axis
	if(fabs(joyYRaw) < deadBand)
	{
		joyYRaw = 0;
	}
	if(joyYRaw + .02 >= joyY && joyYRaw - .02 <= joyY)
	{
		//WAT???????????????
	}
	else if(joyYRaw > joyY)
	{
		joyY += rampRate;
	}
	else
	{
		joyY -= rampRate;
	}

	//Right X axis
	if(fabs(joyRotateRaw) < .1)
	{
		joyRotateRaw = 0;
	}
	if(joyRotateRaw + .02 >= joyRotate && joyRotateRaw - .02 <= joyRotate)
	{
		//WAT???????????????
	}
	else if(joyRotateRaw > joyRotate)
	{
		joyRotate += rampRate;
	}
	else
	{
		joyRotate -= rampRate;
	}
}


void FRC::Mecanum::mecanumDrive(Joystick joy, bool bypass)
{

	double const Pow = 1;
	double xSpeed = joy.GetX();
	double ySpeed = joy.GetY();
	double rotate = joy.GetRawAxis(2);
	if(fabs(rotate) < .1)
	{
		rotation = (.012 * ahrs.GetRate()); //(0.05 * (delta))

		//Mecanum equation(Left side inverted)
		speeds[0] = xSpeed + ySpeed - rotation;//Left Front
		speeds[1] = -(-xSpeed + ySpeed + rotation); //Left Rear
		speeds[2] = -xSpeed + ySpeed - rotation; //Right Front
		speeds[3] = -(xSpeed + ySpeed + rotation); //Right Rear

		//Normalize function: keeps values proportional and below 1
		maxMagnitude = std::fabs(FRC::Mecanum::speeds[0]);
		int i;

		for (i = 1; i < 4; i++)
		{
			double temp = std::fabs(speeds[i]);
			if (maxMagnitude < temp)
			{
				maxMagnitude = temp;
			}
		}
		if (maxMagnitude > 1.0)
		{
			for (i = 0; i < 4; i++)
			{
				speeds[i] = speeds[i] / maxMagnitude;
			}
		}

		if(!bypass)
		{
			// Original
			//EncoderFreq[0] = Left_Front_Motor_Mecanum.GetEncVel(); //Left Front
			//EncoderFreq[1] = -Left_Back_Motor_Mecanum.GetEncVel(); //Left Rear
			//EncoderFreq[2] = Right_Front_Motor_Mecanum.GetEncVel(); //Right Front
			//EncoderFreq[3] = -Right_Back_Motor_Mecanum.GetEncVel(); //Right Rear

			// WPT Modified it
			EncoderFreq[0] = -Left_Front_Motor_Mecanum.GetSelectedSensorVelocity(0);//Left Front
			EncoderFreq[1] = -Left_Back_Motor_Mecanum.GetSelectedSensorVelocity(0); //Left Rear
			EncoderFreq[2] = -Right_Front_Motor_Mecanum.GetSelectedSensorVelocity(0); //Right Front
			EncoderFreq[3] = -Right_Back_Motor_Mecanum.GetSelectedSensorVelocity(0); //Right Rear

			PICorrection(0);
			PICorrection(1);
			PICorrection(2);
			PICorrection(3);

			Left_Front_Motor_Mecanum.Set(PWMout[0] * Pow);
			Left_Back_Motor_Mecanum.Set(PWMout[1] * Pow);
			Right_Front_Motor_Mecanum.Set(PWMout[2] * Pow);
			Right_Back_Motor_Mecanum.Set(PWMout[3] * Pow);
		}
		else
		{
			Left_Front_Motor_Mecanum.Set(speeds[0] * Pow);
			Left_Back_Motor_Mecanum.Set(speeds[1] * Pow);
			Right_Front_Motor_Mecanum.Set(speeds[2] * Pow);
			Right_Back_Motor_Mecanum.Set(speeds[3] * Pow);
		}
	}
	else
	{
		Left_Front_Motor_Mecanum.Set((xSpeed + ySpeed + rotate) * .8);
		Left_Back_Motor_Mecanum.Set(-(-xSpeed + ySpeed - rotate) * .8);
		Right_Front_Motor_Mecanum.Set((-xSpeed + ySpeed + rotate) * .8);
		Right_Back_Motor_Mecanum.Set(-(xSpeed + ySpeed - rotate) * .8);
	}
}


/*Reset Enc and ahrs
 *
 *
 */
