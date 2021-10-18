#include "RPLidarWrapper.h"

Blitz::RPLidarWrapper::RPLidarWrapper()
{
    // Driver
    rplidar = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    u_result rplidar_res = rplidar->connect("/dev/ttyUSB0",115200);
    if (IS_FAIL(rplidar_res))
    {
        cout << "ERROR: Could not connect to RPLidar" << endl;
        return;
    }

    // Device Info
    rplidar_response_device_info_t deviceInfo;
    u_result info_res = rplidar->getDeviceInfo(deviceInfo);
    if (IS_FAIL(info_res))
    {
        cout << "ERROR: Could not get RPLidar device info" << endl;
        return;
    }

    // Device Health
    rplidar_response_device_health_t deviceHealth;
    u_result health_res = rplidar->getHealth(deviceHealth);
    if (IS_FAIL(health_res))
    {
        cout << "ERROR: Could not get RPLidar device health" << endl;
        return;
    }
    else if (deviceHealth.status == RPLIDAR_STATUS_ERROR)
    {
        cout << "ERROR: Please reboot RPLidar" << endl;
        return;
    }

    // Print Serial Number
    cout << "Connected to RPLidar" << endl;

    // Start Scan
    rplidar->startMotor();
	rplidar->startScan(0,1);
}

Blitz::LaserScan Blitz::RPLidarWrapper::GetScan()
{
    LaserScan laserScan = LaserScan();

	rplidar_response_measurement_node_hq_t nodes[8192];
    size_t nodeCount = sizeof(nodes)/sizeof(nodes[0]);
    u_result scan_res = rplidar->grabScanDataHq(nodes, nodeCount);
    if (IS_FAIL(scan_res))
    {
        cout << "Error: Could not get RPLidar scan data" << endl;
        return laserScan;
    }
    rplidar->ascendScanData(nodes,nodeCount);
    
    // Convert Nodes to LaserScan
    for (auto node : nodes)
    {
        float angle = node.angle_z_q14 * 90.f / (1 << 14);
        float distance = (float) node.dist_mm_q2/4.0f/1000;
        int quality = (int)node.quality;

        if (quality < QUALITY_MIN || distance == 0.0)
        {
            continue;
        }

        laserScan.points.push_back(LaserPoint(
            angle, distance, quality
        ));
    }
    return laserScan;
}

GeometryMsgs::Pose Blitz::RPLidarWrapper::GetPose(LaserScan scan)
{
    return rf2o.GetPoseFromLaser(&scan);
}

void Blitz::RPLidarWrapper::Dispose()
{
    rplidar->stop();
    rplidar->stopMotor();
    rplidar->disconnect();
    RPlidarDriver::DisposeDriver(rplidar);
    rplidar = NULL;
}