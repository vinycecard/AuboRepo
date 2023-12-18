#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <deque>
#include <string>
#include <sstream>
#include <tinyxml.h>
#include <iostream>

using namespace std;

deque<double> pos;

int curr_seq_number = -1;
int prev_seq_number = -1;

ros::Time curr_msg_stamp;

//bool spin_callback;

//double sleep_time = .025;

double actual_clock_start;
double actual_clock_time;

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  curr_seq_number = msg->header.seq;
  curr_msg_stamp = msg->header.stamp;

  for (int i = 0; i < msg->position.size(); i++)
    pos.at(i) = msg->position.at(i);

  //spin_callback = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xml_on_the_fly");

  ros::NodeHandle jointstate_nh;
  ros::Subscriber JointState_sub = jointstate_nh.subscribe("joint_states", 1, JointStateCallback);
  ros::Publisher JointState_pub = jointstate_nh.advertise<trajectory_msgs::JointTrajectory>("/joint_command",1);

  trajectory_msgs::JointTrajectory goal;

	goal.joint_names.push_back("joint_s");
	goal.joint_names.push_back("joint_l");
	goal.joint_names.push_back("joint_u");
	goal.joint_names.push_back("joint_r");
	goal.joint_names.push_back("joint_b");
	goal.joint_names.push_back("joint_t");

	int n_joints = 6;
	int dim_point = 3*n_joints + 1;
	int n_points;
	double starting_delta_t = .0253; //.0008;
	double num;
	deque<double> sequence;
	int file_index;

	while (ros::ok())
	{
	 cout << "Enter file number: ";
	 cin >> file_index;

	 ostringstream num_stream;
	 num_stream << file_index;

	 string file_string;

	 //file_string = "/home/lead/Downloads/circle_trajectories/" + num_stream.str();
	 file_string = "/home/lead/Downloads/circles_40hz/" + num_stream.str();

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

	 n_points = sequence.size()/dim_point;

	 goal.points.resize(1);

	 goal.points.at(0).positions.resize(n_joints);
	 goal.points.at(0).velocities.resize(n_joints);
	 goal.points.at(0).accelerations.resize(n_joints);

	 pos.resize(n_joints);

	 do
	 {
		 //spin_callback = true;

		 //while (spin_callback)
			 ros::spinOnce();
	 }
	 while (prev_seq_number == curr_seq_number);

	 for (int i = 0; i < n_joints; i++)
		 goal.points.at(0).positions.at(i) = pos.at(i);

	 goal.points.at(0).time_from_start = ros::Duration(0);

	 JointState_pub.publish(goal);

	 actual_clock_start = ros::Time::now().toSec();
	 actual_clock_time = 0;
	 cout << "actual_clock_time: " << actual_clock_time << " --------- msg_time: " << goal.points.at(0).time_from_start << endl;

	 //ros::Duration(starting_delta_t - 0.025).sleep();

	 for (int i = 0; i < n_points; i++)
	 {
		 do
		 {
		   //spin_callback = true;

		   //while (spin_callback)
	 			 ros::spinOnce();
		 }
		 while (prev_seq_number == curr_seq_number);
		 
		 prev_seq_number = curr_seq_number;
	 	 
		 for (int j = 0; j < n_joints; j++)
		 {
		   goal.points.at(0).positions.at(j) = sequence.at(dim_point*i + j);
		   goal.points.at(0).velocities.at(j) = sequence.at(dim_point*i + j + 6);
		   //goal.points.at(0).accelerations.at(j) = sequence.at(dim_point*i + j + 12);
		 }

		 goal.points.at(0).time_from_start = ros::Duration(starting_delta_t + sequence.at(dim_point*i + 18));

		 JointState_pub.publish(goal);

		 actual_clock_time = ros::Time::now().toSec() - actual_clock_start;
		 cout << "actual_clock_time: " << actual_clock_time << " --------- msg_time: " << goal.points.at(0).time_from_start << endl;
		 
		 //if (sleep_time > 0)
		   //ros::Duration(sleep_time).sleep();
	 }

	 ROS_INFO("Trajectory concluded succesfully.\n");

	 sequence.clear();
	}

  return 0;
}
