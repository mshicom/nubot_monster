#include <ros/ros.h>
#include <string>
#include <nubot_msgs/Cmd.h>
#include <nubot_msgs/Readings.h>
#include <nubot_msgs/Stop.h>
#include <nubot_node/nubot_node.h>
#include <nubot_driver/nubot_base_driver.h>

class Nubot_node
{
public:
	Nubot_node();
	~Nubot_node();

	// 回调函数
	void EncoderCallback(int readings);

private:
	// 回调函数
	void TimerCallback(const ros::TimerEvent& event);
	void CmdCallback(const nubot_msgs::Cmd &msg);
	bool Stop(nubot_msgs::Stop::Request &req, nubot_msgs::Stop::Response &res);

private:
	Nubot_base_driver base_driver;
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Publisher pub;
	ros::Timer serial_timer;
	std::string port;

	// 参数变量
	int SteeringMax, SteeringMin, SteeringBias;
	int SpeedMax, SpeedMin;
	int Freq;

	// 传感器信息
	int encoder1;

	// 指令
	int pluse, rpm;

};

Nubot_node::Nubot_node()
:base_driver(), encoder1(0), pluse(0), rpm(0)
{
	base_driver.readDone = boost::bind(&Nubot_node::EncoderCallback, this, _1);
	n.param<std::string>("port_name", port, "/dev/ttyUSB0");

	// 设置角度和车速限制参数
	n.param<int>("SteeringMax", SteeringMax, 420);
	n.param<int>("SteeringMin", SteeringMin, -280);
	n.param<int>("SteeringBias", SteeringBias, 70);
	n.param<int>("SpeedMax", SpeedMax, 8000);
	n.param<int>("SpeedMin", SpeedMin, -8000);

	// 初始化响应接口
	sub = n.subscribe("nubot_driver/cmd", 1, &Nubot_node::CmdCallback, this);
	pub = n.advertise<nubot_msgs::Readings>("nubot_driver/readings", 1);

	try
	{
		base_driver.connect(port.c_str());
	} catch (Exception &e)
	{
		ROS_FATAL("Fail to Open the port!");
		ROS_BREAK();
	}
	ROS_INFO("Port is opened.");


	// 初始化定时器
	n.param<int>("Control_Frequence", Freq, 30);
	serial_timer = n.createTimer(ros::Duration(1.0 / Freq), &Nubot_node::TimerCallback, this);

	base_driver.Set(0, 0);
}

void Nubot_node::CmdCallback(const nubot_msgs::Cmd &msg)
{
	// 根据传来的速度指令更新节点本地信息
	rpm = msg.speed;
	pluse = msg.angle+SteeringBias;

	// 限幅处理
	if (pluse > SteeringMax)
		pluse = SteeringMax;
	else if (pluse < SteeringMin)
		pluse = SteeringMin;

	if (rpm > 8000)
		rpm = 8000;
	else if (rpm < -8000)
		rpm = -8000;
}

Nubot_node::~Nubot_node()
{
	base_driver.Set(0, 0);
}

void Nubot_node::TimerCallback(const ros::TimerEvent& event)
{
	// 串口通信
	base_driver.Set(pluse, rpm);
}

void Nubot_node::EncoderCallback(int readings)
{
	nubot_msgs::Readings msg;
	msg.encoder1 = encoder1;
	pub.publish(msg);
}

bool Nubot_node::Stop(nubot_msgs::Stop::Request &req,
		nubot_msgs::Stop::Response &res)
{

	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nubot_node");
	Nubot_node nubot;

	// 进入ros循环，等待定时器中断
	ros::spin();

	// 接收到退出信号，关闭程序
	ros::shutdown();
	return 0;
}
