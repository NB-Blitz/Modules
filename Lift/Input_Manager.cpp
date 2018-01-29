#include "WPILib.h"
#include "Input_Manager.hpp"

FRC::Input_Manager::Input_Manager():
	stick(0)
{

}

double FRC::Input_Manager::getAxis(int id)
{
	return stick.GetRawAxis(id);
}
bool FRC::Input_Manager::getButton(int id)
{
	return stick.GetRawButton(id);
}
