#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <industrial_robot_client/joint_trajectory_action.h>
#include <sensor_msgs/JointState.h>

#include <deque>
#include <string>
#include <sstream>
#include <iostream>

#include <tinyxml.h>

using namespace std;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

ros::Time msg_stamp, last_msg_stamp_1, last_msg_stamp_2;

deque<double> pos, last_pos_1, last_pos_2;

double interval;
bool spin_callback, found_begin;

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  msg_stamp = msg->header.stamp;

  for (int i = 0; i < msg->position.size(); i++)
    pos.at(i) = msg->position.at(i);

  spin_callback = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectory_publisher");
  Client client("joint_trajectory_action", true);

  ros::NodeHandle jointstate_nh;

  // Receive encoders messages.
  ros::Subscriber JointState_sub = jointstate_nh.subscribe("joint_states", 1, JointStateCallback);

  // Plot with rqt_plot
  ros::Publisher  JointTrajectory_pub = jointstate_nh.advertise<sensor_msgs::JointState>("goal_trajectory", 2000);
  ros::Publisher  JointVelocity_pub = jointstate_nh.advertise<sensor_msgs::JointState>("joint_velocities", 200);

  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("Client is running.");

  control_msgs::FollowJointTrajectoryGoal goal;

	goal.trajectory.joint_names.push_back("joint_s");
	goal.trajectory.joint_names.push_back("joint_l");
	goal.trajectory.joint_names.push_back("joint_u");
	goal.trajectory.joint_names.push_back("joint_r");
	goal.trajectory.joint_names.push_back("joint_b");
	goal.trajectory.joint_names.push_back("joint_t");

	goal.trajectory.header.frame_id = "/base_link";

	sensor_msgs::JointState joints;
	joints.header.frame_id = "/base_link";

	sensor_msgs::JointState vel;
	vel.header.frame_id = "/base_link";

	int n_joints = 6;
	int point_size = 3*n_joints + 1; // position, velocity and acceleration for each joint + time.
	int n_points;
	double time;
	double starting_delta_t = 2;
	double num;

	deque<double> sequence;

	int n_files = 152;
	int file_index;

	bool grid;

	cout << "grid or circular trajectory? (grid = 1 , circle = 0): ";
	cin >> grid;
	cout << endl;

	cout << "Enter file number: ";
	cin >> file_index;

	while ((file_index >= 0) && (file_index < n_files))
	{
		ostringstream num_stream;
		num_stream << file_index;

		string file_string;

		if (grid)
		 	file_string = "/home/lead/Desktop/Tese-Mestrado/Programas/ROS/Trajetorias_Renan/grid8/" + num_stream.str();

		else
		 	file_string = "/home/lead/Desktop/Tese-Mestrado/Programas/ROS/Trajetorias_Circulares/" + num_stream.str();

		const char* file_name = file_string.c_str();

		cout << file_name << endl;

		TiXmlDocument doc(file_name);
		doc.LoadFile();

		TiXmlElement* trajectory = doc.FirstChildElement("trajectory");
		TiXmlElement* data = trajectory->FirstChildElement("data");

		const char* text = data->GetText();

		string str(text);
		stringstream stream;
		stream << str;

		while (stream >> num) 
			sequence.push_back(num);

		n_points = sequence.size()/point_size + 1;

		goal.trajectory.points.resize(n_points);

		goal.trajectory.points.at(0).positions.resize(n_joints);
		goal.trajectory.points.at(0).velocities.resize(n_joints);
		goal.trajectory.points.at(0).accelerations.resize(n_joints);

		joints.position.resize(n_joints);
		joints.velocity.resize(n_joints);

		spin_callback = true;

		pos.resize(n_joints);
		last_pos_1.resize(n_joints);
		last_pos_2.resize(n_joints);

		while (spin_callback)
			ros::spinOnce();

		for (int i = 0; i < n_joints; i++)
			goal.trajectory.points.at(0).positions.at(i) = pos.at(i);

		time = 0;

		goal.trajectory.points.at(0).time_from_start = ros::Duration(time);
		time += starting_delta_t;

		for (int i = 1; i < n_points; i++)
		{
			goal.trajectory.points.at(i).positions.resize(n_joints);
			goal.trajectory.points.at(i).velocities.resize(n_joints);
			goal.trajectory.points.at(i).accelerations.resize(n_joints);
			
			for (int j = 0; j < n_joints; j++)
			{
				goal.trajectory.points.at(i).positions.at(j) = sequence.at(point_size*(i-1) + j);
		   	goal.trajectory.points.at(i).velocities.at(j) = sequence.at(point_size*(i-1) + j + n_joints);
		   	goal.trajectory.points.at(i).accelerations.at(j) = sequence.at(point_size*(i-1) + j + 2*n_joints);
			}

			if (grid)
			{
				time += sequence.at(point_size*(i-1) + 3*n_joints);
				goal.trajectory.points.at(i).time_from_start = ros::Duration(time);
			}

			else
				goal.trajectory.points.at(i).time_from_start = ros::Duration(time + sequence.at(point_size*(i-1) + 3*n_joints));
		}

		client.sendGoal(goal); // Send trajectory to controller.
		
//------------------------------ Plot trajectory and actual joint velocities on rqt_plot------------------------

		for (int i = 0; i < n_joints; i++)
			joints.position.at(i) = pos.at(i);

		found_begin = false;

		while (!found_begin)
		{
			for (int i = 0; i < n_joints; i++)
				last_pos_1.at(i) = pos.at(i);

			last_msg_stamp_1 = msg_stamp;

			spin_callback = true;

			while (spin_callback)
				ros::spinOnce();

			for (int j = 0; j < n_joints; j++)
			{
				if (last_pos_1.at(j) != pos.at(j)) // Find the moment when the joints actually start moving. 
				{
					found_begin = true;
					break;
				}
			}
		}

		ros::Time begin = last_msg_stamp_1;
		joints.header.stamp = begin;

		JointTrajectory_pub.publish(joints);

		time = starting_delta_t;

		for (int i = 1; i < n_points; i++)
		{
			for (int j = 0; j < n_joints; j++)
			{
				joints.position.at(j) = sequence.at(point_size*(i-1) + j);
				joints.velocity.at(j) = sequence.at(point_size*(i-1) + j + n_joints);
			}
			
			if (grid)
			{
				time += sequence.at(point_size*(i-1) + 3*n_joints);
				joints.header.stamp = begin + ros::Duration(time);
			}

			else
				joints.header.stamp = begin + ros::Duration(time + sequence.at(point_size*(i-1) + 3*n_joints));

			JointTrajectory_pub.publish(joints);
		}

		sequence.clear();

		vel.velocity.resize(n_joints);

		while (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			if (pos.at(0))
			{
				last_msg_stamp_2 = last_msg_stamp_1;
				last_msg_stamp_1 = msg_stamp;

				for (int i = 0; i < n_joints; i++)
				{
					last_pos_2.at(i) = last_pos_1.at(i);
					last_pos_1.at(i) = pos.at(i);
				}
			}

			spin_callback = true;

			while (spin_callback)
				ros::spinOnce();

			interval = msg_stamp.toSec() - last_msg_stamp_2.toSec();

			if (msg_stamp.toSec() > last_msg_stamp_1.toSec())
			{
				vel.header.stamp = last_msg_stamp_1;

				for (int i = 0; i < n_joints; i++)
					vel.velocity.at(i) = (pos.at(i) - last_pos_2.at(i))/interval;

				JointVelocity_pub.publish(vel);
			}
		}
		
//--------------------------------------------------------------------------------------------------------------

		ROS_INFO("Trajectory concluded succesfully.\n");
		cout << "Enter file number: ";
		cin >> file_index;
	}

	ros::shutdown();
	return 0;
}
