#include "BlitzPIDSource.hpp"

#include "WPILib.h"
#include "ctre/Phoenix.h"

frc::BlitzPIDSource::BlitzPIDSource(WPI_TalonSRX &talon)
{
	inputTalon = &talon;

	m_pidSource = PIDSourceType::kRate;
}

double frc::BlitzPIDSource::PIDGet()
{
	return inputTalon->GetSelectedSensorVelocity(0);
}

