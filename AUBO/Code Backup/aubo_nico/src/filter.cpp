#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

using namespace std;
using namespace Eigen;

VectorXd vel(6);
VectorXd last_vel(6);
VectorXd vel_f(6);
VectorXd last_vel_f(6);

int prev_seq_number;
int curr_seq_number;

ros::Time curr_msg_stamp;

double tau = .5;
double h = 1./39.5;

void vel_callback(const sensor_msgs::JointState::ConstPtr& msg)
{
	curr_seq_number = msg->header.seq;
	curr_msg_stamp = msg->header.stamp;

	for (int i = 0; i < 6; i++)
	{
		vel(i) = msg->velocity.at(i);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "filter");
	
	ros::NodeHandle nh;
	
	ros::Subscriber vel_sub = nh.subscribe("/actual_vel", 10, vel_callback);
	ros::Publisher filter_pub = nh.advertise<sensor_msgs::JointState>("/filter_vel",10);
	
	sensor_msgs::JointState filter_msg;
	filter_msg.velocity.resize(6);
	
	for (int i = 0; i < 6; i++)
	{
		last_vel(i) = 0;
		last_vel_f(i) = 0;
	}
	
	prev_seq_number = -1;
	curr_seq_number = -1;
	
	while (ros::ok())
	{
		do
		{
				ros::spinOnce();
		}
		while (prev_seq_number == curr_seq_number);
		
		cout << "curr_seq_number = " << curr_seq_number << endl;
		
		vel_f = (h*(vel+last_vel) + last_vel_f*(2*tau-h))/(2*tau+h);
		
		for (int i = 0; i < 6; i++)
		{
			filter_msg.velocity.at(i) = vel_f(i);
		}
		
		filter_msg.header.stamp = curr_msg_stamp;
		
		filter_pub.publish(filter_msg);
		
		last_vel = vel;
		last_vel_f = vel_f;
		prev_seq_number = curr_seq_number;
	}
}
