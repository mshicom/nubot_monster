/*
 * nubot_teleop_key.cpp
 *
 *  Created on: 2012-10-11
 *      Author: hkh
 */
#include <ros/ros.h>
#include <stdio.h>
#include <pthread.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <nubot_msgs/Cmd.h>
#include <math.h>

class TeleopNubot
{
public:
	TeleopNubot();
	void spinOnce();
	ros::NodeHandle n;
	int vel_mode;
	double angle,speed,delta_speed;

private:
	ros::Publisher vel_pub;
	ros::Subscriber joy_sub,joy_sub2;

	int	SteeringMax, SteeringMin, SteeringBias, SteeringWidth;
	int	SpeedMax, SpeedMin;
	double deadzone_;

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	void joyCallback2(const geometry_msgs::Twist::ConstPtr& joy);
};

TeleopNubot::TeleopNubot()
:vel_mode(false),speed(0),delta_speed(0)
{
	// 设置角度和车速限制参数

	n.param<int>("SteeringMax", SteeringMax, 420);
	n.param<int>("SteeringMin", SteeringMin,  -280);
	n.param<int>("SteeringBias", SteeringBias, 70);
	n.param<int>("SpeedMax", SpeedMax, 8000);
	n.param<int>("SpeedMin", SpeedMin, -8000);

	SteeringWidth = (SteeringMax-SteeringMin)/2;

	vel_pub = n.advertise<nubot_msgs::Cmd>("nubot_driver/cmd", 1);
	joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopNubot::joyCallback, this);
	//joy_sub2 = n.subscribe<geometry_msgs::Twist>("/virtual_joystick/cmd_vel", 10, &TeleopNubot::joyCallback2, this);
}

// 手柄消息回调函数，在ros::spin()里执行
void TeleopNubot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	nubot_msgs::Cmd cmd;
	cmd.angle = SteeringWidth * joy->axes[0];
	angle = cmd.angle;

	if(joy->buttons[2])
		vel_mode=!vel_mode;

	if(vel_mode)
	{
		delta_speed=80*(abs(joy->axes[4])>0.05? joy->axes[4]:0);
	}
	else
	{
		cmd.speed = SpeedMax * joy->axes[4];
		speed = cmd.speed;

		vel_pub.publish(cmd);
	}

}

void TeleopNubot::joyCallback2(const geometry_msgs::Twist::ConstPtr& joy)
{
	nubot_msgs::Cmd cmd;
	double x=joy->linear.x, y=joy->angular.z;

	double ang = atan2(y,x)/3.14159;
	if(fabs(ang)>0.5)
		ang=fabs(ang)*0.5/ang;

	double sign = (x>0)? 1 : -1;

	cmd.angle = SteeringWidth *ang;
	cmd.speed = SpeedMax * sign * hypot(x,y);
	vel_pub.publish(cmd);
}

void TeleopNubot::spinOnce(void)
{
	if(!vel_mode)
		return;

	nubot_msgs::Cmd cmd;

	speed += delta_speed;
	if(speed>SpeedMax)
		speed =  SpeedMax;
	if(speed<SpeedMin)
		speed =  SpeedMin;

	cmd.angle = angle;
	cmd.speed = speed;
	vel_pub.publish(cmd);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nubot_teleop_joy");
	TeleopNubot teleop_nubot;

#if 1
	// 调用joy_node进程,驱动手柄
	pid_t pid = vfork();
	if(pid==0)
	{
		if(execlp("rosrun", "rosrun", "joy", "joy_node", (char *)0) <0)
			ROS_WARN("Process Joy not found!");

		// 正常情况下exec函数不会返回
		return(-1);
	}
#endif
	teleop_nubot.n.setParam("deadzone",0.08);
	ros::Rate loop_rate(30);
	// 等待消息响应
	while(ros::ok())
	{

		teleop_nubot.spinOnce();
		ros::spinOnce();
		loop_rate.sleep();
	}


	// kill(pid, SIGINT);

	ROS_INFO("Joystick thread exit!\n");
	return (0);
}


