#include <stdio.h>      // standard input / output functions
#include <stdlib.h>
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitions
#include <string>
#include <iostream>
#include <mutex>


class radbot_serial
{
	public:
	radbot_serial(std::string serial_port_name)
	{
		USB = open( serial_port_name.c_str(), O_RDWR );
		memset (&tty, 0, sizeof tty);
		if ( tcgetattr ( USB, &tty ) != 0 ) 
		{
			std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
			ros::shutdown();
		}
		cfsetospeed (&tty, (speed_t)B115200);
		cfsetispeed (&tty, (speed_t)B115200);
		tty.c_cflag     &=  ~PARENB;            // Make 8n1
		tty.c_cflag     &=  ~CSTOPB;
		tty.c_cflag     &=  ~CSIZE;
		tty.c_cflag     |=  CS8;

		tty.c_cflag     &=  ~CRTSCTS;           // no flow control
		tty.c_cc[VMIN]   =  1;                  // read doesn't block
		tty.c_cc[VTIME]  =  5;                  // 0.5 seconds read timeout
		tty.c_cflag     |=  CREAD | CLOCAL;     // turn on READ & ignore ctrl lines
		cfmakeraw(&tty);
		tcflush( USB, TCIFLUSH );
		if ( tcsetattr ( USB, TCSANOW, &tty ) != 0) 
		{
			std::cout << "Error " << errno << " from tcsetattr" << std::endl;
			ros::shutdown();
		}
	}
	bool write_port(std::string cmd)
	{
		serial_mutex.lock();
		write(USB, cmd.c_str(), cmd.length());
		serial_mutex.unlock();
	}
	std::string get_command()
	{
		serial_mutex.lock();
		int n = read( USB, &serial_buffer, 64);
		serial_mutex.unlock();
		std::string result = "";
		if (n < 0) {
			ROS_ERROR("Error reading from serial port: %s", strerror(errno));
		}
		else if (n == 0) 
		{
			//ROS_ERROR("Read nothing");
		}
		else 
		{
			//ROS_INFO("Read from serial: %s", serial_buffer);
			result = std::string(serial_buffer);
		}
		return result;
	}
	private:
		std::mutex serial_mutex;
		char serial_buffer[64];
		struct termios tty;
		struct termios tty_old;
		int USB = -1;

};