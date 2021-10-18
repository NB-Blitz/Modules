#pragma once

#include <iostream>
#include "rplidar.h"
#include "LaserScan.h"
#include "GeometryMsgs.h"
#include "RF2O.h"

using namespace rp::standalone::rplidar;
using namespace std;

namespace Blitz
{
	class RPLidarWrapper
	{
		public:
			RPLidarWrapper();

			LaserScan GetScan();
			GeometryMsgs::Pose GetPose(LaserScan scan);
			void Dispose();

		private:
			RPlidarDriver * rplidar;
			RF2O rf2o;

			const int QUALITY_MIN = 10;
	};
}
