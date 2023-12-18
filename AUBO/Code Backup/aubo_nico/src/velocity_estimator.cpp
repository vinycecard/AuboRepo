#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <cmath>

#include <Eigen/Dense>

const int n_joints = 6;
int seq_number, last_seq_number;
double K, Ts;

sensor_msgs::JointState joint_state_estimative;
ros::Subscriber joint_states_sub;
ros::Publisher joint_velocity_pub;

Eigen::VectorXd velocity, theta, last_theta;
ros::Time t, last_t;


void initialize_variables()
{
    K = 1;
    last_t = ros::Time::now();
    last_seq_number = -1;

    joint_state_estimative.position.resize(n_joints);
	joint_state_estimative.velocity.resize(n_joints);
	//joint_state_estimative.accelerations.resize(n_joints);

    //std::cout << "variables initialized" << std::endl;
}

void wait_jointstate()
{
    //std::cout << "wait_jointstate entered" << std::endl;
	do
	{
		ros::spinOnce();
        //std::cout << "seq_number: " << seq_number << " and last_seq_number: " << last_seq_number << std::endl;
        
	}
	while (seq_number == last_seq_number);
}


void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{   
    //std::cout << "callback called" << std::endl;
    joint_state_estimative.header.stamp = msg->header.stamp;
    joint_state_estimative.name = msg->name;
    seq_number = msg->header.seq;
    t = ros::Time::now();
    Ts = (t - last_t).toSec();

	for (int i = 0; i < n_joints; i++)
    {
        theta(i) = msg->position.at(i);
        velocity(i) = (K*(theta(i) - last_theta(i)))/Ts;
        joint_state_estimative.position.at(i) = theta(i);
    }
    //std::cout << "callback finished" << std::endl;
}

void velocity_pub() 
{   
    //std::cout << "velocity_pub entered" << std::endl;

    Eigen::VectorXd::Map(&joint_state_estimative.velocity[0], velocity.size()) = velocity;
    joint_velocity_pub.publish(joint_state_estimative);
    
}

void update_variables()
{
    //std::cout << "update_variables entered" << std::endl;
    last_theta = theta;
    t = last_t;
    last_seq_number = seq_number;

    //std::cout << "variables updated" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_estimator");

    ros::NodeHandle nh;
    
    joint_states_sub = nh.subscribe("/joint_states", 1, &JointStateCallback);
    joint_velocity_pub = nh.advertise<sensor_msgs::JointState>("/joint_velocity",1);

    velocity = Eigen::VectorXd::Zero(n_joints);
    theta = Eigen::VectorXd::Zero(n_joints);
    last_theta = Eigen::VectorXd::Zero(n_joints);

    ros::Duration(1).sleep(); // Time needed to initialize variables

    initialize_variables();

  while (ros::ok())
  {
    wait_jointstate();
    //std::cout << "wait_jointstate exited, seq_number: " << seq_number << std::endl;

    velocity_pub();
    //std::cout << "velocity_pub exited" << std::endl;

    update_variables();
    //std::cout << "update_variables exited" << std::endl;

    //ros::Rate(frequency).sleep();
  }

  return 0;
}