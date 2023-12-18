#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>

#include <iostream>
#include <cmath>

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class JointControl
{

private:
	int n_joints, seq_number, last_seq_number;
	double t, last_t, interval, dt, vel_const;
	bool first_pub;

	VectorXd theta, last_theta, next_theta;
	MatrixXd I, I6;

	ros::Time start_time, pub_start, msg_stamp;

	//-------------------------------------- Topics ------------------------------------------------------

	ros::Subscriber joint_states_sub;

	ros::Publisher joint_command_pub, kin_ctrl_pub, vel_ctrl_pub, actual_vel_pub;
	ros::Publisher ref_position_pub, eff_position_pub, position_error_pub;
	ros::Publisher ref_orientation_pub, eff_orientation_pub, orientation_error_pub;
	
	// Nico: Aubo
	trajectory_msgs::JointTrajectoryPoint waypoint; // replacing 'goal']
	ros::Subscriber moveitcmd_sub;
	MatrixXd moveitTheta, moveitdTheta, moveitddTheta;
	int counter_sub, counter_pub, secs, nsecs;
	//message_filters::Subscriber<sensor_msgs::JointState> msg_filter_sub;
	VectorXd ui, ud; // position input and acceleration input, u (velocity) already declared
	//ros::Rate loop_rate;
	double left_time;
	//
	
	trajectory_msgs::JointTrajectory goal;
	geometry_msgs::PointStamped ref_position, eff_position, position_error;
	geometry_msgs::QuaternionStamped ref_orientation, eff_orientation, orientation_error;
	sensor_msgs::JointState kin_ctrl, vel_ctrl, actual_vel;

	ros::NodeHandle nh;

	//message_filters::Subscriber<sensor_msgs::JointState> msg_filter_sub(nh, "/joint_states", 1);

	//-------------------------------------- Reference ---------------------------------------------------

	int pos_ref_index, quat_ref_index;
	double wp, wo, trace_R, trace_dR, g, dg, theta_d, dtheta_d;

	VectorXd pd, vd, qd, dqd, wd, f, df, u_d, du_d;
	MatrixXd R, dR, E;

	//-------------------------------------- Kinematics -------------------------------------------------------

	// Nico: Aubo
	double pi, a2, a3, d1, d2, d5, d6, ti, alpha_i, ai;

	VectorXd p, q, zq, pqi, pqi_star, pqi_star_temp, p_J, h, h_temp, qi, q_temp;
	MatrixXd DH, P, H, J;

	//-------------------------------------- Kinematic Control -------------------------------------------

	double Kp, Ko;

	VectorXd pe, qe, u_ws, u, theta_min, theta_max, u_max;

	//-------------------------------------- Constant Velocity -------------------------------------------

	int joint, direction;
	double cmd_vel;

	//-------------------------------------- Constant Acceleration ---------------------------------------

	double acc, t0;

	VectorXd u0;

	//-------------------------------------- Velocity Estimative and Control -----------------------------

	double tau, Kp_v, Ki_v;

	VectorXd vel_f, u_c, vel_error, last_vel_error, int_vel_error;

	//----------------------------------------------------------------------------------------------------

  	VectorXd conjugate(const VectorXd& v) // quaternion conjugate (q^(-1)) // q*q^(-1) = [1 0 0 0]
	{
		VectorXd v_conj(4);

		v_conj(0) = v(0);
		v_conj.tail(3) = -v.tail(3);

		return v_conj;
	}

	MatrixXd crossmat(const VectorXd& v) // cross product matrix
	{
		MatrixXd m(3,3);

		m <<     0, -v(2),  v(1),
		      v(2),     0, -v(0),
		     -v(1),  v(0),     0;

		return m;
	}

	VectorXd qdot(const VectorXd& v1, const VectorXd& v2) // quaternion composition
	{
		VectorXd v(4), v1_v(3), v2_v(3);

		v1_v = v1.tail(3);
		v2_v = v2.tail(3);

		v(0) = v1(0)*v2(0) - v1_v.dot(v2_v);
		v.tail(3) = v1(0)*v2_v + v2(0)*v1_v + crossmat(v1_v)*v2_v;

		return v;
	}

	//----------------------------------------------------------------------------------------------------

public:

	JointControl(int argc_, char** argv_): pi(M_PI), n_joints(6), seq_number(-1), last_seq_number(-1), first_pub(true), a2(40.8), a3(37.6), d1(12.2), d2(12.15), d5(10.25), d6(9.4), joint(5), direction(1), u0(0), t0(0), interval(0), counter_pub(1), counter_sub(1)//, loop_rate(10)
	{
		// Receive messages from encoders.
		joint_states_sub = nh.subscribe("/joint_states", 1, &JointControl::JointStateCallback, this);
		// Send trajectory points to controller.
		//joint_command_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/joint_path_command",1);

		// Nico: Aubo
		joint_command_pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/moveItController_cmd",2000); // SELECTED
		//joint_command_pub = nh.advertise<control_msgs::FollowJointTrajectoryActionGoal>("/aubo_i5_controller/follow_joint_trajectory/goal",2000);
		
		// Attempt of receiving trajectory from Rviz and publishing it with a delay -> failed
		//moveitcmd_sub = nh.subscribe("/moveItController_cmd", 1, &JointControl::MoveItCmdCallback, this);
		//message_filters::Subscriber<sensor_msgs::JointState> msg_filter_sub(nh, "/joint_states", 1);
		
		//message_filters::Subscriber<sensor_msgs::JointState> msg_filter_sub(nh, "/joint_states", 1);
		//msg_filter_sub.registerCallback(&JointControl::JointStateCallback, this);

		//message_filters::TimeSequencer<sensor_msgs::JointState> msg_filter_seq(ros::Duration(0.1), ros::Duration(0.01), 10);
		//msg_filter_seq.connectInput(msg_filter_sub);

		
		//msg_filter_sub.registerCallback(&JointControl::JointStateCallback, this);

		
		// Plot with rqt_plot.
		ref_position_pub = nh.advertise<geometry_msgs::PointStamped>("/ref_position",5);
		eff_position_pub = nh.advertise<geometry_msgs::PointStamped>("/eff_position",5);
		position_error_pub = nh.advertise<geometry_msgs::PointStamped>("/position_error",5);
		ref_orientation_pub = nh.advertise<geometry_msgs::QuaternionStamped>("/ref_orientation",5);
		eff_orientation_pub = nh.advertise<geometry_msgs::QuaternionStamped>("/eff_orientation",5);
		orientation_error_pub = nh.advertise<geometry_msgs::QuaternionStamped>("/orientation_error",5);
		kin_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/kin_ctrl",5);
		vel_ctrl_pub = nh.advertise<sensor_msgs::JointState>("/vel_ctrl",5);
		actual_vel_pub = nh.advertise<sensor_msgs::JointState>("/actual_vel",5);

		kin_ctrl.position.resize(n_joints);
		kin_ctrl.velocity.resize(n_joints);   // Kinematic control signal (reference for velocity control).
		vel_ctrl.velocity.resize(n_joints);   // Velocity control signal.
		actual_vel.velocity.resize(n_joints); // Actual joint velocity, estimated from encoders messages.

		goal.points.resize(1);
		goal.points.at(0).positions.resize(n_joints);
		goal.points.at(0).velocities.resize(n_joints);
		goal.points.at(0).accelerations.resize(n_joints);

		// Nico: Aubo
		waypoint.positions.resize(n_joints);
		waypoint.velocities.resize(n_joints);
		waypoint.accelerations.resize(n_joints);

		// Nico: Aubo
		goal.joint_names.push_back("shoulder_joint");
		goal.joint_names.push_back("upperArm_joint");
		goal.joint_names.push_back("foreArm_joint");
		goal.joint_names.push_back("wrist1_joint");
		goal.joint_names.push_back("wrist2_joint");
		goal.joint_names.push_back("wrist3_joint");

		theta.resize(n_joints);
		last_theta.resize(n_joints);
		next_theta.resize(n_joints);

		pd.resize(3);
		qd.resize(4);
		vd.resize(3);
		dqd.resize(4);
		wd.resize(3);
		f.resize(4);
		df.resize(4);
		u_d.resize(3);
		du_d.resize(3);

		R.resize(3,3);
		dR.resize(3,3);
		E.resize(4,3);

		p.resize(3);
		q.resize(4);
		zq.resize(4);
		pqi.resize(4);
		pqi_star.resize(4);
		pqi_star_temp.resize(4);
		p_J.resize(3);
		h.resize(4);
		h_temp.resize(4);
		qi.resize(4);
		q_temp.resize(4);

		DH.resize(n_joints,4);
		P.resize(3,n_joints);
		H.resize(3,n_joints);
		J.resize(6,n_joints);

		pe.resize(3);
		qe.resize(4);
		u_ws.resize(6);
		u.resize(n_joints);
		theta_min.resize(n_joints);
		theta_max.resize(n_joints);
		u_max.resize(n_joints);
		u0.resize(n_joints);

		vel_f.resize(n_joints);
		vel_error.resize(n_joints);
		last_vel_error.resize(n_joints);
		int_vel_error.resize(n_joints);
		u_c.resize(n_joints);

		// Nico: Aubo
		ui.resize(n_joints);
		ud.resize(n_joints);


		// Nico: Aubo
    	// Denavit-Hartenberg Modified matrix (lengths in cm, angles in rad).
		// a_(i-1), alpha_(i-1), d_i, theta_i
		DH <<  	0, 		0, 		d1,		pi,
				0, 		-pi/2,	d2,		-pi/2,
				a2, 	pi, 	0,  	0,
				a3,		pi,  	0,  	-pi/2,
				0,  	-pi/2, 	d5,   	0,
				0,  	pi/2,  	d6,     0;

		zq << 0, 0, 0, 1; // zq = [0; z]

		theta_min << -3.054, -3.054, -3.054, -3.054, -3.054, -3.054; //rad (-175 degrees)
		theta_max << 3.054, 3.054, 3.054, 3.054, 3.054, 3.054;      //rad (175 degrees)
		u_max << 2.618, 2.618, 2.618, pi, pi, pi;                //rad/s (150 / 180 degrees)

		I = MatrixXd::Identity(3,3);
		I6 = MatrixXd::Identity(6,6);

		// See joint_control.launch for definitions.
		if (argc_ == 14)
		{
			pos_ref_index = atoi(argv_[1]);
			quat_ref_index = atoi(argv_[2]);
			wp = atof(argv_[3]);
			wo = atof(argv_[4]);
			Kp = atof(argv_[5]);
			Ko = atof(argv_[6]);
			dt = atof(argv_[7]);
			//loop_rate = (1/atof(argv_[7]));
			vel_const = atof(argv_[8]);
			cmd_vel = atof(argv_[9]);
			acc = atof(argv_[10]);
			tau = atof(argv_[11]);
			Kp_v = atof(argv_[12]);
			Ki_v = atof(argv_[13]);
		}

		ros::Duration(1).sleep();
	}

	~JointControl()
	{
		cout << "Object Destroyed" << endl << endl;
	}

	void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
	{
		seq_number = msg->header.seq;
		msg_stamp = msg->header.stamp; // ROS_TIME (sec.nsec)

		for (int i = 0; i < n_joints; i++)
			theta(i) = msg->position.at(i); //rad
		
	}

	// Nico: Aubo
	// Function would save data received from /moveItController_cmd (Rviz)
	/*
	void MoveItCmdCallback(const trajectory_msgs::JointTrajectoryPoint::ConstPtr& msg)
	{
		moveitTheta.resize(counter_sub,n_joints);
		for (int i = 0; i < n_joints; i++) {
			moveitTheta(counter_sub,i) = msg->positions.at(i); //rad
			moveitTheta(counter_sub,i) = msg->velocities.at(i); //rad
			moveitTheta(counter_sub,i) = msg->accelerations.at(i); //rad
		}
		counter_sub++;
	}
	*/

	/* encoder_update from controlNico.cpp
    void encoder_update_controlNico()
	{
		
        //do
		//{
		//	ros::spinOnce();
		//}
		//while (seq_number == last_seq_number);
        
        
        ros::spinOnce();

        if (seq_number == last_seq_number)
            new_message = false;
        else
            new_message = true;


		if (first_pub)
        {
			start_time = ros::Time::now();
            ui = theta;
            t0 = asin(theta(ARM_DOF-1)/amplitude)/wp;
            last_t = 0;	start_time = ros::Time::now();      // ROS_TIME (sec.nsec)   First encoder message (t = 0).
        }
        // if more than one trajectory, interesting to calculate 't' here
		t = (ros::Time::now() - start_time).toSec(); // Time is updated when new encoder message is received (s).

		//std::cout << std::endl << "-----------------------------------------------------------------" << std::endl << std::endl; 
	}
	*/

	void encoder_update()
	{
		do
		{
			ros::spinOnce();
		}
		while (seq_number == last_seq_number);

		if (first_pub)
			start_time = ros::Time::now();      // ROS_TIME (sec.nsec)   First encoder message (t = 0).

		t = (ros::Time::now() - start_time).toSec(); // Time is updated when new encoder message is received (s).

		//cout << endl << "-----------------------------------------------------------------" << endl << endl; 
	}

	void reference()
	{
		if (pos_ref_index == 0)
		{
			// Nico: Aubo (XZ plane) (Circle centered at Init Position)
			// End-effector position reference (cm). 
			pd(0) = 10*cos(wp*t) - 40.03;
			pd(1) = 0 - 12.15;
			pd(2) = 10*sin(wp*t) + 54.76;

			// End-effector velocity reference (cm/s).
			vd(0) = -10*wp*sin(wp*t);
			vd(1) = 0;
			vd(2) = 10*wp*cos(wp*t);
		}

		else if (pos_ref_index == 1)
		{
			// Nico: Aubo
			// End-effector position reference (cm). (Init Position)
			pd(0) = -40.03;
			pd(1) = -12.15;
			pd(2) = 54.76;

			// End-effector velocity reference (cm/s).
			vd(0) = 0;
			vd(1) = 0;
			vd(2) = 0;
		}

		if (quat_ref_index == 0)
		{
			/* Nico: Aubo (Zero Position)
			// End-effector orientation reference (rotation matrix).
			R(0,0) = 1;  R(0,1) = 0;  R(0,2) = 0;
			R(1,0) = 0;  R(1,1) = 0;  R(1,2) = -1;
			R(2,0) = 0;  R(2,1) = 1;  R(2,2) = 0;
			*/
			
			// Nico: Aubo (Init Position)
			// End-effector orientation reference (rotation matrix).
			R(0,0) = 0;  R(0,1) = -1;  R(0,2) = 0;
			R(1,0) = -1;  R(1,1) = 0;  R(1,2) = 0;
			R(2,0) = 0;  R(2,1) = 0;  R(2,2) = -1;


			dR(0,0) = 0;  dR(0,1) = 0;  dR(0,2) = 0;
			dR(1,0) = 0;  dR(1,1) = 0;  dR(1,2) = 0;
			dR(2,0) = 0;  dR(2,1) = 0;  dR(2,2) = 0;
		}

		else if (quat_ref_index == 1)
		{
			// End-effector orientation reference (angle-axis representation).
			theta_d = pi*sin(wo*t)/2.;
			dtheta_d = pi*wo*cos(wo*t)/2.;

			u_d(0) = 1;
			u_d(1) = 0;
			u_d(2) = 0;

			du_d(0) = 0;
			du_d(1) = 0;
			du_d(2) = 0;
		}

		//------------- Rotation matrix - Quaternion Conversion --------------------
		if (quat_ref_index == 0)
		{
			trace_R = R.trace();
			trace_dR = dR.trace();

			f(0) = trace_R + 1;
			f(1) = R(2,1) - R(1,2);
			f(2) = R(0,2) - R(2,0);
			f(3) = R(1,0) - R(0,1);

			df(0) = trace_dR;
			df(1) = dR(2,1) - dR(1,2);
			df(2) = dR(0,2) - dR(2,0);
			df(3) = dR(1,0) - dR(0,1);

			g = sqrt(trace_R + 1);
			dg = trace_dR/(2.*g);

			qd = f/(2.*g);
			dqd = (df*g-f*dg)/(2.*pow(g,2));
		}

		//--------------- Angle-axis - Quaternion Conversion -----------------------
		else if (quat_ref_index == 1)
		{
			qd(0) = cos(theta_d/2.);
			qd.tail(3) = sin(theta_d/2.)*u_d;

			dqd(0) = -sin(theta_d/2.)*dtheta_d/2.;
			dqd.tail(3) = cos(theta_d/2.)*u_d*dtheta_d/2. + sin(theta_d/2.)*du_d;
		}

		//--------------------------------------------------------------------------

		E.row(0) = -qd.tail(3).transpose(); 
		E.block(1,0,3,3) = qd(0)*I - crossmat(qd.tail(3));

		wd = 2*E.transpose()*dqd;
	}

	void kinematics()
	{  
		q << 1, 0, 0, 0; // End-Effector quaternion.
		p << 0, 0, 0;		 // End-Effector position (cm).

		P = MatrixXd::Constant(3,6,0);
		H = P;

		for (int i = 0; i < n_joints; i++)
		{
		/* 
		  // DH Standard
		  ti = theta(i) + DH(i,3);
		  alpha_i = DH(i,2);
		  ai = DH(i,1);
		

		  pqi(0) = 0;
		  pqi(1) = ai*cos(ti);
		  pqi(2) = ai*sin(ti);
		  pqi(3) = DH(i,0);
		*/

		// Nico: Aubo
		// DH Modified
		  ti = theta(i) + DH(i,3);
		  alpha_i = DH(i,1);
		  ai = DH(i,0);
		
		  pqi(0) = 0;
		  pqi(1) = ai;
		  pqi(2) = -DH(i,2)*sin(alpha_i);
		  pqi(3) = DH(i,2)*cos(alpha_i);


	    pqi_star_temp = qdot(pqi,conjugate(q));
	    pqi_star = qdot(q,pqi_star_temp);

	    P.col(i) = pqi_star.tail(3);
	    p += P.col(i);


		  h_temp = qdot(zq,conjugate(q));
		  h = qdot(q,h_temp);

		  H.col(i) = h.tail(3);

		  qi(0) = cos(ti/2.)*cos(alpha_i/2.);
		  qi(1) = cos(ti/2.)*sin(alpha_i/2.);
		  qi(2) = sin(ti/2.)*sin(alpha_i/2.);
		  qi(3) = sin(ti/2.)*cos(alpha_i/2.);

		  q_temp = qdot(q,qi);
		  q = q_temp;
		}

		p_J = p;

		J.block(3,0,3,6) = H; // End-Effector Geometric Orientation Jacobian.

		for (int i = 0; i < n_joints; i++)
		{
		  J.block(0,i,3,1) = crossmat(H.col(i))*p_J; // End-Effector Geometric Position Jacobian.
		  p_J -= P.col(i);
		}
	}

	void control()
	{
		//-------------------------- Kinematic Control Law -------------------------------------------

		// Errors.
		pe = pd - p;
		qe = qdot(qd,conjugate(q));

		// Workspace Control Signal.
		u_ws.head(3) = vd + Kp*pe;
		u_ws.tail(3) = wd + Ko*qe.tail(3);

	  // Joint Space Control Signal.
		u = J.inverse()*u_ws;
	}

	void sinusoid() 
	{
		ui(n_joints-1) = 0.5*sin(wp*t);
		u(n_joints-1) = 0.5*wp*cos(wp*t);
		ud(n_joints-1) = -0.5*wp*wp*sin(wp*t);
	}

	void square_vel()
	{
		//-------------------------- Square-Wave Velocity ----------------------------------

    	if ((theta(joint) > .8*theta_max(joint)) && (direction == 1))
			direction = -1;

		else if ((theta(joint) < .8*theta_min(joint)) && (direction == -1))
			direction = 1;

		u = direction*cmd_vel*I6.col(joint);
	}

	void triangle_vel()
	{
		//-------------------------- Triangle-Wave Velocity --------------------------------

    	if ((theta(joint) > (theta_max(joint) + theta_min(joint))/2.) && (direction == 1))
		{
			direction = -1;
			u0 = u;
			t0 = last_t;
		}

		else if ((theta(joint) < (theta_max(joint) + theta_min(joint))/2.) && (direction == -1))
		{
			direction = 1;
			u0 = u;
			t0 = last_t;
		}

		u = u0 + direction*acc*(t - t0)*I6.col(joint);
	}

	void velocity_estimative()
	{
		if (!first_pub)
		{	
			interval = t - last_t;

			//vel_f = (theta - last_theta + tau*vel_f)/(interval + tau); // First-order discrete filter (Euler Method)
			vel_f = (2*(theta - last_theta) - (interval - 2*tau)*vel_f)/(interval + 2.*tau); // First-order discrete filter (Tustin Method)
		}
	}

	void velocity_control()
	{
	  last_vel_error = vel_error;
	  vel_error = u - vel_f;
	  //int_vel_error += vel_error*interval;  // Integration (Euler Method)
	  int_vel_error += (vel_error + last_vel_error)*interval/2.;  // Integration (Tustin Method)

	  u_c = u;  // No velocity control.

		//u_c = Kp_v*vel_error;        		// P
		//u_c = u + Kp_v*vel_error;    		// P + FF

		//u_c = Ki_v*int_vel_error; 			// I
		//u_c = u + Ki_v*int_vel_error; 	// I + FF

		//u_c = Kp_v*vel_error + Ki_v*int_vel_error; 			// PI
		//u_c = u + Kp_v*vel_error + Ki_v*int_vel_error; 	  // PI + FF (SELECTED)

		//----------------------------------------------------------------------------------


		// Velocity Control Signal.
		u_c = vel_const*u_c; // (SELECTED)

		// Position and Velocity Joint Limits.
		for (int i = 0; i < n_joints; i++)
		{
		  if (((theta(i) >= .95*theta_max(i)) && (u_c(i) > 0)) || ((theta(i) <= .95*theta_min(i)) && (u_c(i) < 0)))
		    u_c(i) = 0;

		  else if (u_c(i) > u_max(i))
		    u_c(i) = u_max(i);

		  else if (u_c(i) < -u_max(i))
		    u_c(i) = -u_max(i);
		}
	}

	void show_info()
	{
		cout << "seq_number = " << seq_number << endl << endl;
		cout << "t = " << t << endl << endl;
		cout << "theta: " << endl << theta << endl << endl;

		cout << "pd:" << endl << pd << endl << endl;
		cout << "vd:" << endl << vd << endl << endl;
		cout << "qd:" << endl << qd << endl << endl;
		cout << "wd:" << endl << wd << endl << endl;

		cout << "p:" << endl << p << endl << endl;
		cout << "q:" << endl << q << endl << endl;
		cout << "J:" << endl << J << endl << endl;
		cout << "det(J) = " << J.determinant() << endl << endl;
		cout << "inv(J):" << endl << J.inverse() << endl << endl;

		cout << "pe:" << endl << pe << endl << endl;
		cout << "qe:" << endl << qe << endl << endl;
		cout << "u_ws:" << endl << u_ws << endl << endl;
		cout << "u:" << endl << u << endl << endl;

		cout << "interval = " << endl << interval << endl << endl;
		cout << "vel_f:" << endl << vel_f << endl << endl;
		cout << "vel_error:" << endl << vel_error << endl << endl;
		cout << "int_vel_error:" << endl << int_vel_error << endl << endl;
		cout << "u_c:" << endl << u_c << endl << endl;
	}

	void pub_plot_msgs()
	{
		ref_position.point.x = pd(0);
		ref_position.point.y = pd(1);
		ref_position.point.z = pd(2);

		eff_position.point.x = p(0);
		eff_position.point.y = p(1);
		eff_position.point.z = p(2);

		position_error.point.x = pe(0);
		position_error.point.y = pe(1);
		position_error.point.z = pe(2);

		ref_orientation.quaternion.x = qd(0);
		ref_orientation.quaternion.y = qd(1);
		ref_orientation.quaternion.z = qd(2);
		ref_orientation.quaternion.w = qd(3);

		eff_orientation.quaternion.x = q(0);
		eff_orientation.quaternion.y = q(1);
		eff_orientation.quaternion.z = q(2);
		eff_orientation.quaternion.w = q(3);

		orientation_error.quaternion.x = qe(0);
		orientation_error.quaternion.y = qe(1);
		orientation_error.quaternion.z = qe(2);
		orientation_error.quaternion.w = qe(3);

		for (int i = 0; i < n_joints; i++)
		{
		  kin_ctrl.position.at(i) = theta(i);
		  kin_ctrl.velocity.at(i) = u(i);
			vel_ctrl.velocity.at(i) = u_c(i);
			actual_vel.velocity.at(i) = vel_f(i);
		}

		ref_position.header.stamp = msg_stamp;
		eff_position.header.stamp = msg_stamp;
		position_error.header.stamp = msg_stamp;
		ref_orientation.header.stamp = msg_stamp;
		eff_orientation.header.stamp = msg_stamp;
		orientation_error.header.stamp = msg_stamp;
		kin_ctrl.header.stamp = msg_stamp;
		vel_ctrl.header.stamp = msg_stamp;
		actual_vel.header.stamp = msg_stamp;

		ref_position_pub.publish(ref_position);
		eff_position_pub.publish(eff_position);
		position_error_pub.publish(position_error);
		ref_orientation_pub.publish(ref_orientation);
		eff_orientation_pub.publish(eff_orientation);
		orientation_error_pub.publish(orientation_error);
		kin_ctrl_pub.publish(kin_ctrl);
		vel_ctrl_pub.publish(vel_ctrl); 
	  actual_vel_pub.publish(actual_vel);
	}

	void estimative_update()
	{
		last_theta = theta;
		last_seq_number = seq_number;
		last_t = t;

		next_theta = theta + u_c*dt; // Integration (Euler Method). Tustin cannot be used as it would require the next value of u_c.
	}

	// Nico: Aubo
	// Not used
	void pub_copied_msg() {
		if (first_pub)
		{
			first_pub = false;

			//waypoint.time_from_start = ros::Duration(0);
			//waypoint.time_from_start = ros::Time::now().toSec;

			pub_start = ros::Time::now();

			// joint_command_pub.publish(waypoint);
		}	

		for (int j = 0; j < n_joints; j++)
		{
			waypoint.positions.at(j) = moveitTheta(counter_pub,j);
			waypoint.velocities.at(j) = moveitdTheta(counter_pub,j);
			waypoint.accelerations.at(j) = moveitddTheta(counter_pub,j);
		}

		//waypoint.time_from_start = (ros::Time::now() - pub_start) + ros::Duration(dt);
		//waypoint.time_from_start = ros::Time::now().toSec + ros::Duration(dt);
		joint_command_pub.publish(waypoint);
		
		counter_pub++;

		// Slightly increases estimated velocity noise, but greatly reduces CPU consumption. 
		ros::Duration(.5*dt).sleep();
	}


	void pub_traj_msg()
	{
		//int secs, nsecs;
		if (first_pub)
		{
			first_pub = false;

			for (int j = 0; j < n_joints; j++)
			{				
				goal.points.at(0).positions.at(j) = theta(j); //commented
				goal.points.at(0).velocities.at(j) = 0; //commented
				goal.points.at(0).accelerations.at(j) = 0; //commented

				// Nico: Aubo
				waypoint.positions.at(j) = theta(j);
				waypoint.velocities.at(j) = 0;
			}

			goal.points.at(0).time_from_start = ros::Duration(0); //commented

			//Nico: Aubo
			waypoint.time_from_start = ros::Duration(0);
			//waypoint.time_from_start.sec = int (ros::Time::now().toSec);
			//waypoint.time_from_start.nsec = int (ros::Time::now().toNSec);


			pub_start = ros::Time::now();
			//joint_command_pub.publish(goal);

			// Nico: Aubo
			joint_command_pub.publish(waypoint);
			//goalpoint.goal.trajectory = goal;
			//joint_command_pub.publish(goalpoint);
		}

		//left_time = dt - loop_rate.cycleTime().toSec();	
		//dt - ros::Rate(1/dt).cycleTime().toSec();

		for (int j = 0; j < n_joints; j++)
		{
			//goal.points.at(0).positions.at(j) = next_theta(j);
			//goal.points.at(0).velocities.at(j) = u_c(j);

			// Nico: Aubo
			/*
			waypoint.positions.at(j) = next_theta(j);
			waypoint.velocities.at(j) = u_c(j);
			*/

			// Nico: Aubo (Sinusoid)
			
			waypoint.positions.at(j) = ui(j) + u(j)*dt;
			waypoint.velocities.at(j) = (u(j) + ud(j)*dt);
			waypoint.accelerations.at(j) = ud(j);

			goal.points.at(0).positions.at(j) = ui(j) + u(j)*(dt - ros::Rate(1/dt).cycleTime().toSec());
			goal.points.at(0).velocities.at(j) = (u(j) + ud(j)*(dt - ros::Rate(1/dt).cycleTime().toSec()));
			goal.points.at(0).accelerations.at(j) = (ud(j));
			
		}
		/*
		if (counter_pub > 5) {
			counter_pub = 0;
			pub_start = ros::Time::now();
			} 	counter_pub++;
		*/

		goal.points.at(0).time_from_start = (ros::Time::now() - pub_start) + ros::Duration(dt);
		//goal.points.at(0).time_from_start = ros::Duration(dt); //commented
		//goalpoint.header.stamp = msg_stamp;
		//goalpoint.goal_id.stamp = msg_stamp;
		//goalpoint.goal_id.id = "joint_control_node";
		//goalpoint.goal.trajectory = goal;
		//goalpoint.goal.trajectory.header.frame_id = "/world";
		
		//joint_command_pub.publish(goal);

		// Nico: Aubo
		waypoint.time_from_start = (ros::Time::now() - pub_start) + ros::Duration(dt);
		//waypoint.time_from_start.sec = int ((ros::Time::now() + ros::Duration(dt)).toSec);
		//waypoint.time_from_start.nsec = int ((ros::Time::now() + ros::Duration(dt)).toNSec);
		
		joint_command_pub.publish(waypoint); // SELECTED
		
		// Slightly increases estimated velocity noise, but greatly reduces CPU consumption. 
		//ros::Duration(.5*dt).sleep();
	
	}

	void execute()
	{
		while (ros::ok())
		{
			encoder_update();

		 	//reference();
			//kinematics();
			//control();

			sinusoid();
			//square_vel();
			//triangle_vel();

			//velocity_estimative(); // SELECTED
			//velocity_control(); // SELECTED

			// show_info();
			// pub_plot_msgs();

			//estimative_update();
			pub_traj_msg(); // SELECTED
			ros::Rate(1/dt).sleep();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_control");
	JointControl joint_control(argc, argv);
	joint_control.execute();

	return 0;
}
