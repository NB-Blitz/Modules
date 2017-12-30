#include <WPILib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public frc::SampleRobot {

public:
	Robot() {
	}


	void OperatorControl() {

		//The socket stuff should be in its own class.  The RobotInit should attempt to do
		//the port connection stuff.  Like the python code, this needs to have error checking
		//and attempt to redo connections and the like if it loses connection.  We also need to make
		//sure that socket communications don't block if a connection goes down or gets hosed up.
		//As the code sits right now, it definitely will block if it can't connect.  The same thing applies
		//to the python code.

		//As it sits right now, it does provide an easier way to test communications because you can go
		//in and out of teleOp mode and see how things respond.

		int counter = 0;

		sockaddr_in m_address;
		unsigned short m_port = 50007;

		int m_sock = socket(AF_INET, SOCK_STREAM, 0);
		m_address.sin_family = AF_INET;
	    m_address.sin_port = htons(m_port);

	    inet_pton(AF_INET, "169.254.3.121", &m_address.sin_addr.s_addr);

	    connect(m_sock, (struct sockaddr *)&m_address, sizeof(m_address));


		while (IsOperatorControl() && IsEnabled()) {

			char msg[] = "Distance";
		    int msg_len = strlen(msg);
		    send(m_sock, msg, msg_len, 0);


   		    char* buffer = static_cast<char*>(malloc(50 * sizeof(char)));

			int receviedBytes = recv(m_sock, buffer, 50, 0);

			SmartDashboard::PutString("Returned string", std::string(buffer, receviedBytes));
			SmartDashboard::PutNumber("Bytes returned", receviedBytes);

			SmartDashboard::PutNumber("Counter", counter);
			counter++;

			frc::Wait(0.005);			// wait for a motor update time
		}

	    close(m_sock);
	}
};

START_ROBOT_CLASS(Robot)
