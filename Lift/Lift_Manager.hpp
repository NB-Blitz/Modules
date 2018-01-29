#ifndef SRC_LIFT_MANAGER_HPP_
#define SRC_LIFT_MANAGER_HPP_

#include "WPILib.h"
#include "ctre/Phoenix.h"

namespace FRC
{
	class Lift_Manager
	{
	public:
		Lift_Manager();

		// Objects
		WPI_TalonSRX Lift_Motor; // Lift Motor

		// Variables
		double speedRef = 0;      // Raw Motor Output
		double positionError = 0; // Distance Between current position and target position
		double positionRef = 0;   // Target Position

		const double UP_GAIN         = 0.00025;  // Speed Multiplier when going up
		const double DOWN_GAIN       = 0.000125; // Speed Multiplier when going down
		const double LOWER_SPEED_MIN = -0.2;     // Minimum Speed (When going down)
		const double UPPER_SPEED_MIN = 0.35;     // Minimum Speed (When going up)

		const int MAX_LIFT_POS = 15000; // Maximum encoder counts
		const int WINDOW       = 200;   // If within this distance to our target, stop

		// Methods
		void moveLiftTo(double pos);  // Moves Lift to pos (Between -1 and 1)
		void moveLift(double stickY); // Set Lift Motor to stickY
		void resetLift();             // TO BE CREATED!!! Auto-Calibrates Lift
		void resetEnc();              // Sets Encoders to 0
		void updateSD();              // Updates SmartDashboard
	};
}

#endif
