#include "OpenCVWrapper.h"

Blitz::OpenCVWrapper::OpenCVWrapper(string streamName) :
    cam(0)
{
    cvsource = frc::CameraServer::GetInstance()->PutVideo(streamName, WIDTH, HEIGHT);

    if (!cam.isOpened())
    {
        cout << "Error: Could not connect to Camera" << endl;
        return;
    }
    if (!cvsource.IsConnected())
    {
        cout << "Error: Could not connect to Camera Server" << endl;
        return;
    }
    
    Upload(Mat::zeros(HEIGHT, WIDTH, CV_8UC3));
    cout << "Connected " << streamName << " to Camera Server" << endl;
}

void Blitz::OpenCVWrapper::Draw(LaserScan& scan, GeometryMsgs::Pose pose)
{
    Mat frame = Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

    // Laserscan
    for (LaserPoint p : scan.points)
    {
        int x = (int)(LASER_SCALE * p.distance * cos(p.angle * (PI/180.0)) + (WIDTH / 2));
        int y = (int)(LASER_SCALE * p.distance * sin(p.angle * (PI/180.0)) + (HEIGHT / 2));
        circle(frame, Point(x, y), 2, Scalar(0,0,255), FILLED);
    }

    // Crosshair
    line(frame, Point(0, HEIGHT / 2), Point(WIDTH, HEIGHT / 2), Scalar(150, 150, 150));
    line(frame, Point(WIDTH / 2, 0), Point(WIDTH / 2, HEIGHT), Scalar(150, 150, 150));

    // Robot
    circle(frame, Point((int)(pose.position.x * POS_SCALE) + (WIDTH / 2), (int)(pose.position.y * POS_SCALE) + (HEIGHT / 2)), 5, Scalar(255, 255, 255), FILLED);
    
    // Camera Server
    Upload(frame);
}
void Blitz::OpenCVWrapper::Draw(Point2f ball)
{
    Mat frame = Mat::zeros(HEIGHT, WIDTH, CV_8UC3);

    // Ball
    circle(frame, ball, 8, Scalar(50,100,100), FILLED);

    // Crosshair
    line(frame, Point(0, HEIGHT / 2), Point(WIDTH, HEIGHT / 2), Scalar(150, 150, 150));
    line(frame, Point(WIDTH / 2, 0), Point(WIDTH / 2, HEIGHT), Scalar(150, 150, 150));

    // Camera Server
    Upload(frame);
}

Point2f Blitz::OpenCVWrapper::FindClosestBall(Scalar HSVlow, Scalar HSVhigh)
{
    // Input
    Mat frame = GetCamFrame();

    // HSV Threshold
    inRange(frame, HSVlow, HSVhigh, frame);
    
    // Blob Detector
    SimpleBlobDetector::Params blobParams;
    blobParams.filterByCircularity = true;
    blobParams.minCircularity = 0.1;

    SimpleBlobDetector* detector = SimpleBlobDetector::create(blobParams);
    std::vector<KeyPoint> keypoints;
	detector->detect(frame, keypoints);
    drawKeypoints(frame, keypoints, frame, Scalar(255,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    // Average 
    Point2f avg(
        keypoints[0].pt.x,
        keypoints[0].pt.y
    );
    avg.x -= WIDTH / 2;
    avg.x -= HEIGHT / 2;
    avg.x /= WIDTH / 2;
    avg.y /= HEIGHT / 2;

    // Draw
    Upload(frame);
    return avg;
}

void Blitz::OpenCVWrapper::Upload(Mat frame)
{
    cvsource.PutFrame(frame);
}

Mat Blitz::OpenCVWrapper::GetCamFrame()
{
    Mat frame;
    cam >> frame;
    return frame;
}