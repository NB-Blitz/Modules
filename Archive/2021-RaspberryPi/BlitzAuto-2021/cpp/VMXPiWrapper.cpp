#include "VMXPiWrapper.h"

Blitz::VMXPiWrapper::VMXPiWrapper() :
    vmx(false, 50)
{
    if (!vmx.IsOpen()) {
		cout << "ERROR: Could not connect to VMXPi" << endl;
        return;
	}
    cout << "Connected to VMXPi" << endl;
    vmx.setPerformanceLogging(false);
    vmx.ahrs.ZeroYaw();
}

float Blitz::VMXPiWrapper::GetYaw()
{
    return vmx.ahrs.GetYaw() + 180.0;
}

float Blitz::VMXPiWrapper::GetYawVelocity()
{
    return vmx.ahrs.GetRate();
}