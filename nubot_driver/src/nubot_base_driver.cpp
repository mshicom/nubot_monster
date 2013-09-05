/*
 * Nubot_base_driver.cpp
 *
 *  Created on: 2012-10-11
 *      Author: 黄开宏
 */

#include <stdio.h>
#include <string>
#include <errno.h>
#include <termios.h>
#include <fcntl.h>
#include <stdexcept>

#include <nubot_driver/nubot_base_driver.h>

#define Baud_Rate 115200

//! Macro for throwing an exception with a message, passing args
#define NUBOT_EXCEPT(except, msg, ...) \
{ \
    char buf[1000]; \
    snprintf(buf, 1000, msg " (in Port::%s)" , ##__VA_ARGS__, __FUNCTION__); \
    throw except(buf); \
}

Nubot_base_driver::Nubot_base_driver()
: readDone(NULL), fd_(-1), stream_thread_(0), stream_stopped_(true), encoder1(0), data_updated(false)
{
}

Nubot_base_driver::~Nubot_base_driver()
{
	stream_stopped_ = true;
	stream_thread_->join();

	delete stream_thread_;
	stream_thread_ = NULL;

	if(portOpen()) close();
}

void Nubot_base_driver::close()
{
	int retval = ::close(fd_);
  	fd_ = -1;
  	if(retval != 0)
    		NUBOT_EXCEPT(Exception, "Failed to close port properly -- error = %d: %s\n", errno, strerror(errno));
}

void Nubot_base_driver::Set(int pluse, int rpm)
{
	// 往串口发命令，不带标志位
	data_out.pluse = pluse;
	data_out.rpm = rpm;

	if(::write(fd_, (const char*) &data_out, sizeof(DataToSerial)) < 0)
		perror("Writing failed");
}

void Nubot_base_driver::SetPluse(int pluse)
{
	data_out.pluse = pluse;

	if(::write(fd_, (const char*) &data_out, sizeof(DataToSerial)) < 0)
		perror("Writing failed");
}

void Nubot_base_driver::SetRpm(int rpm)
{
	data_out.rpm = rpm;
	if(::write(fd_, (const char*) &data_out, sizeof(DataToSerial)) < 0)
			perror("Writing failed");
}

void Nubot_base_driver::readThread()
{
	char flag;									// 标志位
	DataFromSerial data;
	char * buffer = (char*)&data;
	int ret=0;
	unsigned int current;

	while(!stream_stopped_  && ret != -1)
	{
		// 对齐数据,找数据头: 0xAA
		do{
			ret = ::read(fd_, &flag, 1);
			if(ret <0 )	break;
		}
		while(flag!=-86);

		// 读有效数据
		for(current=0; current < sizeof(DataFromSerial); current += ret)
		{
			ret = ::read(fd_, &buffer[current], sizeof(DataFromSerial)-current);
			if(ret == -1)	break;
		}

		// 找数据尾: 0xFF
		ret = ::read(fd_, &flag, 1);
		if(ret <0 )	break;
		if(flag==-1)
		{
			// 数据有效
			encoder1 = data.encoder1;
			data_updated = true;

			if(readDone!=NULL)
				readDone(encoder1);
		}
	}
}

bool Nubot_base_driver::GetEncoder(int &data)
{
	data = encoder1;
	if(data_updated)
	{
		data_updated = false;
		return true;
	}
	else
		return false;
}

void Nubot_base_driver::connect(const char* port_name)
{
		if(portOpen()) close();

		fd_ = ::open(port_name, O_RDWR | O_NOCTTY);

		if(fd_ == -1)
		{
			const char *extra_msg = "";
			switch(errno)
			{
				case EACCES:
				extra_msg = "You probably don't have permission to open the port for reading and writing.";
				break;

				case ENOENT:
				extra_msg = "The requested port does not exist. ";
				break;
			}
			NUBOT_EXCEPT(Exception, "Failed to open port: %s. %s (errno = %d). %s", port_name, strerror(errno), errno, extra_msg);
		}

		try
		{
			struct flock fl;
			fl.l_type = F_WRLCK;
			fl.l_whence = SEEK_SET;
			fl.l_start = 0;
			fl.l_len = 0;
			fl.l_pid = getpid();

			if(fcntl(fd_, F_SETLK, &fl) != 0)
				NUBOT_EXCEPT(Exception, "Device %s is already locked. Try 'lsof | grep %s' to find other processes that currently have the port open.", port_name, port_name);

			// 设置串口模式
			struct termios newtio;
			tcgetattr(fd_, &newtio);
			memset (&newtio.c_cc, 0, sizeof (newtio.c_cc));
			newtio.c_cflag = CS8 | CLOCAL | CREAD;		// 8 bits,Ignore modem status lines, Allow input to be received
			newtio.c_iflag = IGNPAR;									// Ignore characters with parity errors
			newtio.c_oflag = 0;
			newtio.c_lflag = 0;
			cfsetspeed(&newtio, Baud_Rate);

			// Activate new settings
			tcflush(fd_, TCIFLUSH);
			if(tcsetattr(fd_, TCSANOW, &newtio) < 0)
				NUBOT_EXCEPT(Exception, "Unable to set serial port attributes. The port you specified (%s) may not be a serial port.", port_name); /// @todo tcsetattr returns true if at least one attribute was set. Hence, we might not have set everything on success.
			usleep (200000);

			if(portOpen())
			{
				stream_thread_ = new boost::thread(boost::bind(&Nubot_base_driver::readThread, this));
				stream_stopped_ = false;
			}
		}
		catch(Exception& e)
		{
			// These exceptions mean something failed on open and we should close
			if(fd_ != -1) ::close(fd_);
			fd_ = -1;
			throw e;
		}
}
