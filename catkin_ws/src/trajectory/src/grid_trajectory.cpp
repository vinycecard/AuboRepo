#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <industrial_robot_client/joint_trajectory_action.h>
#include <sensor_msgs/JointState.h>

#include <deque>
#include <string>
#include <sstream>
#include <tinyxml.h>
#include <iostream>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;

std::deque<double> pos;
std::deque<double> prev_pos_1;
std::deque<double> prev_pos_2;

ros::Time curr_msg_stamp;
ros::Time prev_msg_stamp_1;
ros::Time prev_msg_stamp_2;

double interval;

bool spin_callback;
bool found_begin;

void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  curr_msg_stamp = msg->header.stamp;

  for (int i = 0; i < msg->position.size(); i++)
    pos[i] = msg->position[i];

  spin_callback = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "grid_trajectory");
  Client client("joint_trajectory_action", true);

  ROS_INFO("Waiting for action server to start.");
  client.waitForServer();
  ROS_INFO("Client is running.");

  ros::NodeHandle jointstate_nh;
  ros::Subscriber JointState_sub = jointstate_nh.subscribe("joint_states", 1, JointStateCallback);
  ros::Publisher  JointTrajectory_pub = jointstate_nh.advertise<sensor_msgs::JointState>("goal_trajectory", 2000);
  ros::Publisher  JointVelocity_pub = jointstate_nh.advertise<sensor_msgs::JointState>("joint_velocities", 200);

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

  
  goal.trajectory.header.frame_id = "/base_link";

  sensor_msgs::JointState joints;
  joints.header.frame_id = "/base_link";

  sensor_msgs::JointState vel;
  vel.header.frame_id = "/base_link";

  int n_joints = 6;
  int dim_point = 3*n_joints + 1;
  int n_points;
  double time = 0;
  double starting_delta_t = 2;
  double num;
  std::deque<double> sequence;

  int n_files = 152;

  for (int file_index = 0; file_index < n_files; file_index++)
  {
     std::ostringstream num_stream;
     num_stream << file_index;
     std::string file_string("/home/lead/Downloads/grid8/" + num_stream.str());

     const char* file_name = file_string.c_str();
   
     TiXmlDocument doc(file_name);
     doc.LoadFile();

     TiXmlElement* trajectory = doc.FirstChildElement("trajectory");
     TiXmlElement* data = trajectory->FirstChildElement("data");

     const char* text = data->GetText();

     std::string str(text);
     std::stringstream stream;
     stream << str;

     while (stream >> num) 
       sequence.push_back(num);
  }

     n_points = sequence.size()/dim_point + 1;

     goal.trajectory.points.resize(n_points);

     goal.trajectory.points[0].positions.resize(n_joints);
     goal.trajectory.points[0].velocities.resize(n_joints);
     goal.trajectory.points[0].accelerations.resize(n_joints);

     joints.position.resize(n_joints);
     joints.velocity.resize(n_joints);

     spin_callback = true;

     while (spin_callback)
       ros::spinOnce();

     for (int i = 0; i < n_joints; i++)
       goal.trajectory.points[0].positions[i] = pos[i];

     goal.trajectory.points[0].time_from_start = ros::Duration(time);
     time += starting_delta_t;

     for (int i = 1; i < n_points; i++)
     {
       goal.trajectory.points[i].positions.resize(n_joints);
       goal.trajectory.points[i].velocities.resize(n_joints);
       goal.trajectory.points[i].accelerations.resize(n_joints);

       goal.trajectory.points[i].positions[0] = sequence[dim_point*(i-1)];
       goal.trajectory.points[i].positions[1] = sequence[dim_point*(i-1) + 1];
       goal.trajectory.points[i].positions[2] = sequence[dim_point*(i-1) + 2];
       goal.trajectory.points[i].positions[3] = sequence[dim_point*(i-1) + 3];
       goal.trajectory.points[i].positions[4] = sequence[dim_point*(i-1) + 4];
       goal.trajectory.points[i].positions[5] = sequence[dim_point*(i-1) + 5];
       goal.trajectory.points[i].velocities[0] = sequence[dim_point*(i-1) + 6];
       goal.trajectory.points[i].velocities[1] = sequence[dim_point*(i-1) + 7];
       goal.trajectory.points[i].velocities[2] = sequence[dim_point*(i-1) + 8];
       goal.trajectory.points[i].velocities[3] = sequence[dim_point*(i-1) + 9];
       goal.trajectory.points[i].velocities[4] = sequence[dim_point*(i-1) + 10];
       goal.trajectory.points[i].velocities[5] = sequence[dim_point*(i-1) + 11];
       goal.trajectory.points[i].accelerations[0] = sequence[dim_point*(i-1) + 12];
       goal.trajectory.points[i].accelerations[1] = sequence[dim_point*(i-1) + 13];
       goal.trajectory.points[i].accelerations[2] = sequence[dim_point*(i-1) + 14];
       goal.trajectory.points[i].accelerations[3] = sequence[dim_point*(i-1) + 15];
       goal.trajectory.points[i].accelerations[4] = sequence[dim_point*(i-1) + 16];
       goal.trajectory.points[i].accelerations[5] = sequence[dim_point*(i-1) + 17];

       time += sequence[dim_point*(i-1) + 18];
       goal.trajectory.points[i].time_from_start = ros::Duration(time);
     }

     client.sendGoal(goal);

     for (int i = 0; i < n_joints; i++)
       joints.position[i] = pos[i];

             found_begin = false;

             while (!found_begin)
             {
               for (int i = 0; i < n_joints; i++)
                 prev_pos_1[i] = pos[i];

               prev_msg_stamp_1 = curr_msg_stamp;

               spin_callback = true;

	       while (spin_callback)
	         ros::spinOnce();

               for (int j = 0; j < n_joints; j++)
               {
                 if (prev_pos_1[j] != pos[j])
                 {
                   found_begin = true;
                   break;
                 }
               }
             }

             ros::Time begin = prev_msg_stamp_1;
             joints.header.stamp = begin;


     JointTrajectory_pub.publish(joints);

     time = starting_delta_t;

     for (int i = 1; i < n_points; i++)
     {
       joints.position[0] = sequence[dim_point*(i-1)];
       joints.position[1] = sequence[dim_point*(i-1) + 1];
       joints.position[2] = sequence[dim_point*(i-1) + 2];
       joints.position[3] = sequence[dim_point*(i-1) + 3];
       joints.position[4] = sequence[dim_point*(i-1) + 4];
       joints.position[5] = sequence[dim_point*(i-1) + 5];
       joints.velocity[0] = sequence[dim_point*(i-1) + 6];
       joints.velocity[1] = sequence[dim_point*(i-1) + 7];
       joints.velocity[2] = sequence[dim_point*(i-1) + 8];
       joints.velocity[3] = sequence[dim_point*(i-1) + 9];
       joints.velocity[4] = sequence[dim_point*(i-1) + 10];
       joints.velocity[5] = sequence[dim_point*(i-1) + 11];

       time += sequence[dim_point*(i-1) + 18];

       joints.header.stamp = begin + ros::Duration(time);

       JointTrajectory_pub.publish(joints);
     }

     vel.velocity.resize(n_joints);

     while (client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
     {
       if (pos[0])
       {
         prev_msg_stamp_2 = prev_msg_stamp_1;
         prev_msg_stamp_1 = curr_msg_stamp;

         for (int i = 0; i < n_joints; i++)
         {
           prev_pos_2[i] = prev_pos_1[i];
           prev_pos_1[i] = pos[i];
         }
       }

       spin_callback = true;

       while (spin_callback)
         ros::spinOnce();

       interval = curr_msg_stamp.toSec() - prev_msg_stamp_2.toSec();

       if (curr_msg_stamp.toSec() > prev_msg_stamp_1.toSec())
       {
         vel.header.stamp = prev_msg_stamp_1;

         for (int i = 0; i < n_joints; i++)
           vel.velocity[i] = (pos[i] - prev_pos_2[i])/interval;

         JointVelocity_pub.publish(vel);
       }
     }

  ROS_INFO("Trajectory concluded succesfully.");

  return 0;
}
