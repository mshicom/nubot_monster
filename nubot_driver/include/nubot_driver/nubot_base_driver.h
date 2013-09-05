#ifndef _NUBOT_BASE_DRIVER_INCLUDED
#define _NUBOT_BASE_DRIVER_INCLUDED


#include <boost/function.hpp>
#include <boost/thread/thread.hpp>


// 串口数据结构，需要与单片机的程序保持一致
typedef struct {
	int pluse;
	int rpm;
} DataToSerial;

typedef struct {
	int encoder1;
} DataFromSerial;

class Nubot_base_driver
{
public:
	Nubot_base_driver();
	~Nubot_base_driver();
	boost::function<void(int)> readDone;

	void connect(const char* port_name);
	bool GetEncoder(int &data);
	void Set(int pluse, int rpm);
	void SetPluse(int pluse);
	void SetRpm(int rpm);

	//! Close the serial port
	void close();

	//! Check whether the port is open or not
	bool portOpen() { return fd_ != -1; }

private:
	void readThread();


private:
	//! File descriptor
	int fd_;

	//! Stream thread
	boost::thread * stream_thread_;
	bool 					stream_stopped_;

	DataToSerial 		data_out;

	int 		encoder1;
	volatile bool 	data_updated;

};

//! Macro for defining an exception with a given parent (std::runtime_error should be top parent)
#define DEF_EXCEPTION(name, parent) \
		class name : public parent { \
		public: \
		name(const char* msg) : parent(msg) {} \
}

//! A standard exception
DEF_EXCEPTION(Exception, std::runtime_error);

#undef DEF_EXCEPTION

#endif //_NUBOT_BASE_DRIVER_INCLUDED
