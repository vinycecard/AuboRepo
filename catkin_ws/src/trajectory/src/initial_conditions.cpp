#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <industrial_robot_client/joint_trajectory_action.h>
#include <sensor_msgs/JointState.h>

#include <deque>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

deque<double> theta;

int n_joints = 6;
int seq_number = -1;

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	seq_number = msg->header.seq;

  for (int i = 0; i < n_joints; i++)
    theta.at(i) = msg->position.at(i);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "initial_conditions");
  Client client("joint_trajectory_action", true);

  ros::NodeHandle jointstate_nh;
  ros::Subscriber JointState_sub = jointstate_nh.subscribe("joint_states", 1, JointStateCallback);
  
  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("Client is running.");

  control_msgs::FollowJointTrajectoryGoal goal;

	/*
	goal.trajectory.joint_names.push_back("joint_s");
	goal.trajectory.joint_names.push_back("joint_l");
	goal.trajectory.joint_names.push_back("joint_u");
	goal.trajectory.joint_names.push_back("joint_r");
	goal.trajectory.joint_names.push_back("joint_b");
	goal.trajectory.joint_names.push_back("joint_t");
	*/

	// Nico: Aubo
	goal.trajectory.joint_names.push_back("shoulder_joint");
	goal.trajectory.joint_names.push_back("upperArm_joint");
	goal.trajectory.joint_names.push_back("foreArm_joint");
	goal.trajectory.joint_names.push_back("wrist1_joint");
	goal.trajectory.joint_names.push_back("wrist2_joint");
	goal.trajectory.joint_names.push_back("wrist3_joint");

	
	
	goal.trajectory.points.resize(2);

	goal.trajectory.points.at(0).positions.resize(n_joints);
	goal.trajectory.points.at(0).velocities.resize(n_joints);

	goal.trajectory.points.at(1).positions.resize(n_joints);
	goal.trajectory.points.at(1).velocities.resize(n_joints);

	theta.resize(n_joints);

	do
	{
		ros::spinOnce();
	}
	while (seq_number == -1);

	for (int i = 0; i < n_joints; i++)
		goal.trajectory.points.at(0).positions.at(i) = theta.at(i);

	goal.trajectory.points.at(0).time_from_start = ros::Duration(0);

	/*
  // Initial joint conditions for circular trajectory
	goal.trajectory.points.at(1).positions.at(0) = 0.4636;
	goal.trajectory.points.at(1).positions.at(1) = 0.5408;
	goal.trajectory.points.at(1).positions.at(2) = -0.0282;
	goal.trajectory.points.at(1).positions.at(3) = 0;
	goal.trajectory.points.at(1).positions.at(4) = -1.0018;
	goal.trajectory.points.at(1).positions.at(5) = -0.4636;

  // Initial joint conditions for 6th joint test
	/*goal.trajectory.points.at(1).positions.at(0) = 0;
	goal.trajectory.points.at(1).positions.at(1) = 0;
	goal.trajectory.points.at(1).positions.at(2) = 0;
	goal.trajectory.points.at(1).positions.at(3) = 0;
	goal.trajectory.points.at(1).positions.at(4) = 0;
	goal.trajectory.points.at(1).positions.at(5) = -6.283;
	*/
	
    // Nico: Aubo
  // Initial joint conditions for circular trajectory
	goal.trajectory.points.at(1).positions.at(0) = 0;
	goal.trajectory.points.at(1).positions.at(1) = -0.1273*0;
	goal.trajectory.points.at(1).positions.at(2) = -1.3212*0;
	goal.trajectory.points.at(1).positions.at(3) = 0.3769*0;
	goal.trajectory.points.at(1).positions.at(4) = -1.5708*0;
	goal.trajectory.points.at(1).positions.at(5) = 0;

  // Initial joint conditions for 6th joint test
	/*goal.trajectory.points.at(1).positions.at(0) = 0;
	goal.trajectory.points.at(1).positions.at(1) = 0;
	goal.trajectory.points.at(1).positions.at(2) = 0;
	goal.trajectory.points.at(1).positions.at(3) = 0;
	goal.trajectory.points.at(1).positions.at(4) = 0;
	goal.trajectory.points.at(1).positions.at(5) = -6.283;*/

	goal.trajectory.points.at(1).time_from_start = ros::Duration(2);

	client.sendGoal(goal);
}
