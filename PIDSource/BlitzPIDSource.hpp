#pragma once

#include "WPILib.h"
#include "ctre/Phoenix.h"

namespace frc {

class BlitzPIDSource : public PIDSource
{

	public:
		WPI_TalonSRX *inputTalon;

		BlitzPIDSource(WPI_TalonSRX &talon);

		double PIDGet();

};

}  // namespace frc
