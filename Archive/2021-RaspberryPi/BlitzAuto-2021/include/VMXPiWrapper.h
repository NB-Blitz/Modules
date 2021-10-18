#pragma once

#include <iostream>
#include "VMXPi.h"

using namespace std;

namespace Blitz
{
	class VMXPiWrapper
	{
		public:
			VMXPiWrapper();

			float GetYaw();
			float GetYawVelocity();

		private:
			VMXPi vmx;
	};
}