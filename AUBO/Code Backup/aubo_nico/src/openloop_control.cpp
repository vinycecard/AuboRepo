#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>

#include <Eigen/Dense>

//#include "otg/otgnewslib.h"
//#include "aubo_driver/aubo_driver.h"
//#include "aubo_driver.cpp"

//using namespace Eigen;

// Global Parameters
int n_joints, trajectory, target_topic, square_direction;
double t, wp, frequency, dt, amplitude, left_time;
bool first_pub;

Eigen::VectorXd u, ui, ud, udd;

ros::Publisher joint_command_pub;
ros::Time pub_start, start_time;
trajectory_msgs::JointTrajectoryPoint waypoint;
trajectory_msgs::JointTrajectory goal;


void sinusoid() 
{
  if (first_pub) 
		start_time = ros::Time::now();

	t = (ros::Time::now() - start_time).toSec();

	ui(n_joints-1) = amplitude*sin(wp*t);
	u(n_joints-1) = amplitude*wp*cos(wp*t);
	ud(n_joints-1) = -amplitude*wp*wp*sin(wp*t);
  udd(n_joints-1) = -amplitude*wp*wp*wp*cos(wp*t);
}

void squarewave()
{
  if (first_pub) 
		start_time = ros::Time::now();

	t = (ros::Time::now() - start_time).toSec();
  
  if (fmod(t,10) < 5)
  {
    square_direction = 1;
    ui(n_joints-1) = wp*fmod(t,10)*square_direction;
  }
  else
  { 
    square_direction = -1;
    ui(n_joints-1) = wp*(10-fmod(t,10))*-square_direction;
  }

	//ui(n_joints-1) = amplitude*fmod(t,10)*square_direction;
	u(n_joints-1) = amplitude*square_direction;
	//ud(n_joints-1) = -amplitude*wp*wp*sin(wp*t)*0;
  //udd(n_joints-1) = -amplitude*wp*wp*wp*cos(wp*t)*0;


}

void pub_traj_msg()
{ 
  if (first_pub)
	{
		first_pub = false;
    
    std::vector<double> zerovect(n_joints, 0);

    waypoint.positions = zerovect;
    waypoint.velocities = zerovect;
    waypoint.accelerations = zerovect;
    
    waypoint.time_from_start = ros::Duration(0.0);


    if (target_topic == 0) 
      joint_command_pub.publish(waypoint);

    else if (target_topic == 1) 
    {
      goal.points.at(0) = waypoint;
      joint_command_pub.publish(goal);
    }

		pub_start = ros::Time::now();

	}

	left_time = dt - ros::Rate(frequency).cycleTime().toSec();	

  // Conversion from Eigen vector to std vector
  Eigen::VectorXd::Map(&waypoint.positions[0], ui.size()) = ui;
  Eigen::VectorXd::Map(&waypoint.velocities[0], u.size()) = u;
  Eigen::VectorXd::Map(&waypoint.accelerations[0], ud.size()) = ud;

  waypoint.time_from_start = (ros::Time::now() - pub_start) + ros::Duration(left_time);


  if (target_topic == 0) 
    joint_command_pub.publish(waypoint);
  
  else if (target_topic == 1) 
  {
    goal.points.at(0) = waypoint;
    joint_command_pub.publish(goal);
  }
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "openloop_control");

  ros::NodeHandle nh;

  if (argc == 6)
	{
    amplitude = atof(argv[1]);
		wp = atof(argv[2]);
    trajectory = atoi(argv[3]);
    target_topic = atoi(argv[4]);
		frequency = atof(argv[5]);
	}

  if (target_topic == 0) 
    joint_command_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/moveItController_cmd",1);

  else if (target_topic == 1) 
    joint_command_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command",2000);
  
  
  dt = 1/frequency;
  n_joints = 6;
  first_pub = true;
  square_direction = 1;
    
  u = Eigen::VectorXd::Zero(n_joints);
  ui = Eigen::VectorXd::Zero(n_joints);
  ud = Eigen::VectorXd::Zero(n_joints);
  udd = Eigen::VectorXd::Zero(n_joints);

	waypoint.positions.resize(n_joints);
	waypoint.velocities.resize(n_joints);
	waypoint.accelerations.resize(n_joints);

  if (target_topic == 1)
  {
    goal.points.resize(1);
    goal.points.at(0) = waypoint;

	  goal.joint_names.push_back("shoulder_joint");
	  goal.joint_names.push_back("upperArm_joint");
	  goal.joint_names.push_back("foreArm_joint");
	  goal.joint_names.push_back("wrist1_joint");
	  goal.joint_names.push_back("wrist2_joint");
	  goal.joint_names.push_back("wrist3_joint");
  }

  ros::Duration(1).sleep(); // Time needed to initialize variables


  while (ros::ok())
  {
    if (trajectory == 0)
      sinusoid();
    else if (trajectory == 1)
      squarewave();
    
    pub_traj_msg();

    ros::spinOnce();

    ros::Rate(frequency).sleep();
  }

  return 0;
}

