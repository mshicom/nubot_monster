/*
 * nubot_teleop_key.cpp
 *
 *  Created on: 2012-10-11
 *      Author: hkh
 */
#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <nubot_msgs/Cmd.h>

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

int kfd = 0;
struct termios cooked, raw;

class TeleopNubot
{
public:
	TeleopNubot();
	void keyLoop();
	char keyGet();
	void keyInit();

private:
	ros::NodeHandle n;
	ros::Publisher vel_pub;

	int step;
	int angle, speed;


	int	SteeringMax, SteeringMin, SteeringBias, SteeringWidth;
	int	SpeedMax, SpeedMin;

};

TeleopNubot::TeleopNubot() :
step(40),angle(0), speed(0)
{
	// 设置角度和车速限制参数
	n.param<int>("SteeringMax", SteeringMax, 420);
	n.param<int>("SteeringMin", SteeringMin,  -280);
	n.param<int>("SteeringBias", SteeringBias, 70);
	n.param<int>("SpeedMax", SpeedMax, 8000);
	n.param<int>("SpeedMin", SpeedMin, -8000);

	SteeringWidth = (SteeringMax-SteeringMin)/2;

	vel_pub = n.advertise<nubot_msgs::Cmd>("nubot_driver/cmd", 1);
}

// 进程中止回调函数
void quit(int sig)
{
	// 还原命令行设置模式
	tcsetattr(kfd, TCSANOW, &cooked);
	ros::shutdown();
	exit(0);
}

// 设置命令行模式
void TeleopNubot::keyInit()
{
	// get the console in raw mode
	tcgetattr(kfd, &cooked);
	memcpy(&raw, &cooked, sizeof(struct termios));
	raw.c_lflag &= ~(ICANON | ECHO);

	// Setting a new line, then end of file
	raw.c_cc[VEOL] = 1;
	raw.c_cc[VEOF] = 2;
	tcsetattr(kfd, TCSANOW, &raw);

	puts("Reading from keyboard");
	puts("---------------------------");
	puts("Use arrow keys to move the robot. Space ->stop; R ->Reset");

	signal(SIGINT, quit);
}

// 阻塞模式下读取按键输入
char TeleopNubot::keyGet()
{
	char c;
	// get the next event from the keyboard
	if (read(kfd, &c, 1) < 0)
	{
		perror("read():");
		exit(-1);
	}
	return c;
}

// 处理按键的消息循环
void TeleopNubot::keyLoop()
{
	bool dirty = false;
	for (;;)
	{
		// ROS_DEBUG("value: 0x%02X\n", c);
		switch (keyGet())
		{
		case KEYCODE_L:
			angle += step;
			dirty = true;
			break;
		case KEYCODE_R:
			angle -= step;
			dirty = true;
			break;
		case KEYCODE_U:
			speed += 50;
			dirty = true;
			break;
		case KEYCODE_D:
			speed -= 50;
			dirty = true;
			break;
		case ' ':
			speed = 0;
			dirty = true;
			break;
		case 'r':
			angle = 0;
			dirty = true;
			break;
		}

		// 限幅处理
		angle = (angle > SteeringWidth) ? SteeringWidth :
						(angle < -SteeringWidth) ? -SteeringWidth :	angle;

		speed = (speed > SpeedMax) ? SpeedMax :
						(speed < SpeedMin) ? SpeedMin :	speed;

		if (dirty  && ros::ok())
		{
			nubot_msgs::Cmd cmd;
			cmd.angle = angle;
			cmd.speed = speed;
			vel_pub.publish(cmd);

			dirty = false;
		}
	}
	return;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "nubot_teleop_key");
	TeleopNubot teleop_nubot;

	//键盘主线程，keyLoop会一直阻塞直至退出
	teleop_nubot.keyInit();
	teleop_nubot.keyLoop();

	return (0);
}


