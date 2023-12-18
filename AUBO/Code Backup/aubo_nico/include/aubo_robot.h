#ifndef AUBO_DRIVER_H_
#define AUBO_DRIVER_H_

#include "ros/ros.h"
#include <cmath>

#include <Eigen/Dense>

//#include "../aubo_driver/aubo_driver.h"

#define pi M_PI

class JointControl
{
private:
	// Aubo functions return (0 = success)
	// int ret;

    // ROS
    ros::NodeHandle nh;

    /*
	// Aubo positions
    double zero_position[ARM_DOF] = {0};
    double initial_position[ARM_DOF] = {0.0/180.0*pi,  -7.2917/180.0*pi,  -75.6946/180.0*pi, 21.5967/180.0*pi, -90.0/180.0*pi, 0.0/180.0*pi};
	double initial_position2[ARM_DOF] = {-90.0/180.0*pi,  -7.2917/180.0*pi,  -75.6946/180.0*pi, 21.5967/180.0*pi, -90.0/180.0*pi, 0.0/180.0*pi};
	double initial_position3[ARM_DOF] = {90.0/180.0*pi,  -7.2917/180.0*pi,  -75.6946/180.0*pi, 21.5967/180.0*pi, -90.0/180.0*pi, 0.0/180.0*pi};
	double initial_position_bluechair[ARM_DOF] = {0.0/180.0*pi,  14.8137/180.0*pi,  -134.2134/180.0*pi, -59.0270/180.0*pi, -90.0/180.0*pi, 0.0/180.0*pi};
	double initial_position_stairs[ARM_DOF] = {0.0/180.0*pi,  0.7939/180.0*pi,  -128.2163/180.0*pi, -39.0101/180.0*pi, -90.0/180.0*pi, 0.0/180.0*pi};
	
    // double t
	double wp, interval, last_t;
    //bool first_pub;

	// Control
    Eigen::VectorXd joint_pos_target, joint_vel_target, joint_pos, last_joint_pos, joint_vel;
	Eigen::VectorXd delta_joint_pos;
	
	// Trajectory
	int sampling_time = 0;
	double tmax, points_length;
	Eigen::MatrixXd joint_traj;
	Eigen::VectorXd joint_pos_initial;
	
	//-------------------------------------- Reference ---------------------------------------------------
	double wo, trace_R, trace_dR, g, dg, theta_d, dtheta_d;
	int joint_selected = 3;

	Eigen::VectorXd pd, vd, qd, dqd, wd, f, df, u_d, du_d;
	Eigen::MatrixXd R, dR, E;

	//-------------------------------------- Kinematics -------------------------------------------------------
	Eigen::VectorXd p, q;
	Eigen::MatrixXd DH, P, H, J, Jtool;
	Eigen::Vector3d pne;
	Eigen::Matrix3d Rne;
	const Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3,3);
	const Eigen::Matrix3d Zeros3 = Eigen::Matrix3d::Zero();
	const Eigen::MatrixXd I6 = Eigen::MatrixXd::Identity(6,6);

	//-------------------------------------- Kinematic Control -------------------------------------------
	double Kp, Ko;

	Eigen::VectorXd pe, qe, u_ws, joint_pos_min, joint_pos_max, joint_vel_max;

    // ------------------ useful functions ----------------

  	Eigen::VectorXd conjugate(const Eigen::VectorXd& v) // quaternion conjugate (q^(-1)) // q*q^(-1) = [1 0 0 0]
	{
		Eigen::VectorXd v_conj(4);

		v_conj(0) = v(0);
		v_conj.tail(3) = -v.tail(3);

		return v_conj;
	}

	Eigen::MatrixXd crossmat(const Eigen::VectorXd& v) // cross product matrix
	{
		Eigen::MatrixXd m(3,3);

		m <<     0, -v(2),  v(1),
		      v(2),     0, -v(0),
		     -v(1),  v(0),     0;

		return m;
	}

	Eigen::VectorXd qdot(const Eigen::VectorXd& v1, const Eigen::VectorXd& v2) // quaternion composition
	{
		Eigen::VectorXd v(4), v1_v(3), v2_v(3);

		v1_v = v1.tail(3);
		v2_v = v2.tail(3);

		v(0) = v1(0)*v2(0) - v1_v.dot(v2_v);
		v.tail(3) = v1(0)*v2_v + v2(0)*v1_v + crossmat(v1_v)*v2_v;

		return v;
	}

	Eigen::VectorXd rotatePos(const Eigen::VectorXd& position, const Eigen::VectorXd& quaternion) // rotate position vector using quaternion
	{
		Eigen::VectorXd new_position(3), position_aux(4);

		position_aux << 0, position(0), position(1), position(2);

		position_aux = qdot(position_aux, conjugate(quaternion));
		position_aux = qdot(quaternion, position_aux);

		new_position = position_aux.tail(3); // new_position = q*position*q^-1

		return new_position;
	}

	Eigen::VectorXd rotm2quat(const Eigen::MatrixXd& rotationMatrix) // Rotation Matrix to Quaternion Conversion
	{
		// Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
		Eigen::VectorXd quaternion(4);
  		float trace = rotationMatrix.trace();
  		if( trace > 0 ) 
		{
    		float s = 0.5f / sqrtf(trace+ 1.0f);
    		quaternion(0) = 0.25f / s;
    		quaternion(1) = ( rotationMatrix(2,1) - rotationMatrix(1,2) ) * s;
    		quaternion(2) = ( rotationMatrix(0,2) - rotationMatrix(2,0) ) * s;
    		quaternion(3) = ( rotationMatrix(1,0) - rotationMatrix(0,1) ) * s;
  		} else {
    		if ( rotationMatrix(0,0) > rotationMatrix(1,1) && rotationMatrix(0,0) > rotationMatrix(2,2) ) {
      			float s = 2.0f * sqrtf( 1.0f + rotationMatrix(0,0) - rotationMatrix(1,1) - rotationMatrix(2,2));
      			quaternion(0) = (rotationMatrix(2,1) - rotationMatrix(1,2) ) / s;
      			quaternion(1) = 0.25f * s;
      			quaternion(2) = (rotationMatrix(0,1) + rotationMatrix(1,0) ) / s;
      			quaternion(3) = (rotationMatrix(0,2) + rotationMatrix(2,0) ) / s;
    		} else if (rotationMatrix(1,1) > rotationMatrix(2,2)) {
      			float s = 2.0f * sqrtf( 1.0f + rotationMatrix(1,1) - rotationMatrix(0,0) - rotationMatrix(2,2));
      			quaternion(0) = (rotationMatrix(0,2) - rotationMatrix(2,0) ) / s;
      			quaternion(1) = (rotationMatrix(0,1) + rotationMatrix(1,0) ) / s;
      			quaternion(2) = 0.25f * s;
      			quaternion(3) = (rotationMatrix(1,2) + rotationMatrix(2,1) ) / s;
    		} else {
      			float s = 2.0f * sqrtf( 1.0f + rotationMatrix(2,2) - rotationMatrix(0,0) - rotationMatrix(1,1) );
      			quaternion(0) = (rotationMatrix(1,0) - rotationMatrix(0,1) ) / s;
      			quaternion(1) = (rotationMatrix(0,2) + rotationMatrix(2,0) ) / s;
      			quaternion(2) = (rotationMatrix(1,2) + rotationMatrix(2,1) ) / s;
      			quaternion(3) = 0.25f * s;
    		}
  		}
		  return quaternion;
	}

	Eigen::Matrix3d quat2rotm(const Eigen::VectorXd& quaternion) // Quaternion to Rotation Matrix Conversion
	{
		Eigen::Matrix3d rotationMatrix;
		double q0 = quaternion(0);
		Eigen::Vector3d qv = quaternion.tail(3);
		rotationMatrix = (2*(q0*q0)-1)*I + 2*(qv*qv.transpose() + q0*crossmat(qv));

		return rotationMatrix;
	} */

public:
	int ret;
    JointControl();
    /*
    JointControl(int argc_, char** argv_): wp(1.0), tmax(50.0),
	frequency(150.0), 
	//timinglog("/AUBO/logs/") 
	timinglog("/Dropbox/Projeto Final/logs/")
    {
        // ROS Subscribers
		record_Sub = nh.subscribe<std_msgs::Bool>("/timinglog_record", 5, &JointControl::RecordCallback, this);
        force_Sub = nh.subscribe<geometry_msgs::WrenchStamped>("/netft_data", 1, &JointControl::ForceCallback, this);
		
		// Trajectory initialization
		points_length = tmax*frequency;
        joint_pos_initial = Eigen::VectorXd::Zero(ARM_DOF);
        joint_traj = Eigen::MatrixXd::Zero(points_length,ARM_DOF);

        joint_pos_target = Eigen::VectorXd::Zero(ARM_DOF);
        joint_vel_target = Eigen::VectorXd::Zero(ARM_DOF);
		delta_joint_pos = Eigen::VectorXd::Zero(ARM_DOF);
        joint_pos = Eigen::VectorXd::Zero(ARM_DOF);
        last_joint_pos = Eigen::VectorXd::Zero(ARM_DOF);

        joint_vel = Eigen::VectorXd::Zero(ARM_DOF);

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

		DH.resize(ARM_DOF,8);
		P.resize(3,ARM_DOF);
		H.resize(3,ARM_DOF);
		J.resize(6,ARM_DOF);
		Jtool.resize(6,ARM_DOF);

		pe.resize(3);
		qe.resize(4);
		u_ws.resize(6);
		joint_pos_min.resize(ARM_DOF);
		joint_pos_max.resize(ARM_DOF);
		joint_vel_max.resize(ARM_DOF);

        Kp = .5; // .5
        Ko = 0;
		
		force_ref << 0, 0, -8;

		Kpf = (-1./600)*I;

		Tiinv = (.25)*I;

		Kel = (-600)*I;

		// peq << -.1215, .4003, .3415;
		peq << -.1215, .4003, .5476;


		// Denavit-Hartenberg Modified parameters 
		// (lengths in [m], angles in [rad])
		double a2, a3, d1, d2, d5, d6;
		a2 = (.408); a3 = (.376); d1 = (.0985);
		d2 = (.1215); d5 = (.1025); d6 = (.094);
		double theta1, theta2, theta4, alpha1, alpha2, alpha3, alpha4, alpha5;
		theta1 = pi; theta2 = -pi/2; theta4 = -pi/2;
		alpha1 = -pi/2; alpha2 = pi; alpha3 = pi; alpha4 = -pi/2; alpha5 = pi/2;

		// Matrix
		// sines and cosines precalculated to increase efficiency.
		// cai = cos(alpha(i-1)), cai2 = cos(alpha(i-1)/2);
		// sai = sin(alpha(i-1)), sai2 = sin(alpha(i-1)/2);

		// a_(i-1), alpha_(i-1), d_i,   theta_i, 	cai, 	sai,		cai2,			sai2,
		DH <<  	0, 		0, 		d1,		theta1,		1, 		0,			1, 				0,
				0, 		alpha1,	d2,		theta2,  	0,		-1,			cos(-pi/4), 	sin(-pi/4),
				a2, 	alpha2, 0,  	0,			-1,		0,			0,				1,
				a3,		alpha3, 0,  	theta4,		-1,		0,			0,				1,
				0,  	alpha4, d5,   	0,			0,		-1,			cos(-pi/4),		sin(-pi/4),
				0,  	alpha5, d6,     0,			0,		1,			cos(pi/4),		sin(pi/4);

		// tool
		pne << 0.0, 0.0, 0.135;
		Rne = I;

		// Limits
		joint_pos_min << -3.054, -3.054, -3.054, -3.054, -3.054, -3.054; //rad (-175 degrees)
		joint_pos_max << 3.054, 3.054, 3.054, 3.054, 3.054, 3.054;      //rad (175 degrees)
		//joint_vel_max << 2.618, 2.618, 2.618, pi, pi, pi;                //rad/s (150 / 180 degrees)

		// I = Eigen::MatrixXd::Identity(3,3);
		// I6 = Eigen::MatrixXd::Identity(6,6);
		// Zeros3 = Eigen::Matrix3d::Zero();

		// Eigen::Matrix3d Zeros3 = Eigen::Matrix3d::Zero();
		// Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    }
    */

    ~JointControl();
	// {   
		//std::cout << std::endl << std::endl << "JointControl Object Destroyed" << std::endl;
		//std::cout << "Teach Pendant ready to use" << std::endl << std::endl;

		//double cycletime_avg = cycletime/count;
		//std::cout << "Cycletime average = " << cycletime_avg << " secs = ";
		//std::cout << 1/cycletime_avg << " hz" << std::endl;
	// }

    // void InitializeDriver(aubo_driver::AuboDriver &robot_driver) 
    // {
    //     ret = robot_driver.robot_send_service_.robotServiceLeaveTcp2CanbusMode();
    //     if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    //         ROS_ERROR("Failed to leave Tcp2Canbus Mode, error code:%d", ret);
    
    //     // Robot move to zero position
    //     ret = robot_driver.robot_send_service_.robotServiceJointMove(initial_position_stairs, true);
    //     if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    //         ROS_ERROR("Failed to move to initial position, error code:%d", ret);

    //     ros::Duration(.5).sleep();

    //     ret = robot_driver.robot_send_service_.robotServiceEnterTcp2CanbusMode();
    //     if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    //         ROS_ERROR("Failed to enter Tcp2Canbus Mode, error code:%d", ret);         
    // }
    void InitializeDriver();

	// Eigen::VectorXd getJointAngle(aubo_driver::AuboDriver &robot_driver)
    // {
	// 	Eigen::VectorXd joint_angle(ARM_DOF);

	// 	aubo_robot_namespace::JointParam jointParam;

    //     ret = robot_driver.robot_send_service_.robotServiceGetJointAngleInfo(jointParam);
	// 	if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
    //         ROS_ERROR("Failed to get Joint Angle Info, error code:%d", ret);

	// 	for (int i = 0; i < ARM_DOF; i++)
	// 		joint_angle(i) = jointParam.jointPos[i];

	// 	return joint_angle;
    // }
    Eigen::VectorXd getJointAngle();

/*
	void kinematics(Eigen::VectorXd& joint_pos)
	{  
		Eigen::Vector3d zq(0.0, 0.0, 1.0);
		q << 1, 0, 0, 0; // End-Effector quaternion.
		p << 0, 0, 0;		 // End-Effector position (cm).

		P = Eigen::MatrixXd::Constant(3,6,0);
		H = P;

		for (int i = 0; i < ARM_DOF; i++)
		{
		  	// DH Modified
		  	double theta_i = joint_pos(i) + DH(i,3);
		  	double alpha_i = DH(i,1);
		  	double ai = DH(i,0);
		  	double di = DH(i,2);

		  	double cti2 = cos(theta_i/2);
		  	double sti2 = sin(theta_i/2);
		  
		  	double cai = DH(i,4);
		  	double sai = DH(i,5);
		  	double cai2 = DH(i,6);
		  	double sai2 = DH(i,7);

			Eigen::Vector3d pqi;
			pqi(0) = ai;
			pqi(1) = -di*sai;
			pqi(2) = di*cai;

			P.col(i) = rotatePos(pqi, q);

	    	p += P.col(i);

			Eigen::Vector4d qi;
			qi(0) = cti2*cai2;
			qi(1) = cti2*sai2;
			qi(2) = -sti2*sai2;
			qi(3) = sti2*cai2;

			q = qdot(q,qi);

			H.col(i) = rotatePos(zq,q);
		}

		Eigen::Vector3d p_J(p(0), p(1), p(2));

		Eigen::Matrix3d R0n = quat2rotm(q);
		//std::cout << "Ron = \n" << R0n << std::endl;
		
		Jtool << I, -crossmat(R0n*pne), Zeros3, I; // Still gotta test

        // matrix.block(i,j,p,q) = Block of size (p,q), starting at (i,j)
        // bottom 3 rows defined
		J.block(3,0,3,6) = H; // End-Effector Geometric Orientation Jacobian.

		for (int i = 0; i < ARM_DOF; i++)
		{
			p_J -= P.col(i);
		  	J.block(0,i,3,1) = crossmat(H.col(i))*p_J; // End-Effector Geometric Position Jacobian.
		}

		//
		//std::cout << "Jtool*J = \n" << Jtool*J << std::endl;

	}

	void checklimits() // Check Joint limit (|3.054| rad = |175| deg) and Check deltaJointPosition limit (|0.01| rad)
	{
		for (int i = 0; i < ARM_DOF; i++)
		{
			if (((joint_pos(i) >= .95*joint_pos_max(i)) && (joint_pos_target(i) > 0)) || ((joint_pos(i) <= .95*joint_pos_min(i)) && (joint_pos_target(i) < 0)))
		    	joint_pos_target(i) = joint_pos(i);
			else if (joint_pos_target(i) - joint_pos(i) > 0.01) // Check deltaJointPosition limit (0.01 rad)
		  		joint_pos_target(i) = joint_pos(i) + 0.01;
			else if (joint_pos_target(i) - joint_pos(i) < -0.01) // Check deltaJointPosition limit (-0.01 rad)
		  		joint_pos_target(i) = joint_pos(i) - 0.01;
		}
	}

    void sendJointPos(aubo_driver::AuboDriver &robot_driver)
    {
		if (sampling_time < points_length) {
		double position_sent[ARM_DOF];
		joint_pos_target = joint_traj.row(sampling_time);
        memcpy(position_sent, joint_pos_target.data(), sizeof(double) * ARM_DOF);

        ret = robot_driver.robot_send_service_.robotServiceSetRobotPosData2Canbus(position_sent); 
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to move to position_sent, error code:%d", ret);
		}
		sampling_time++;
    }

	void sendJointPosOnline(aubo_driver::AuboDriver &robot_driver)
    {
		// joint_pos_target = joint_pos + delta_joint_pos; // IF ONLINE (BAD PERFORMANCE)
		joint_pos_target += delta_joint_pos; // IF OFFLINE (TEST)
		double position_sent[ARM_DOF];
        memcpy(position_sent, joint_pos_target.data(), sizeof(double) * ARM_DOF);

        ret = robot_driver.robot_send_service_.robotServiceSetRobotPosData2Canbus(position_sent); 
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to move to position_sent, error code:%d", ret);
    }
*/
/*
	void execute_online() // online traj (Force is simulated)
	{
        aubo_driver::AuboDriver robot_driver;

		std::string serverHostParameter;
    	ros::param::get("/aubo_driver/server_host", serverHostParameter);
		ros::param::set("/aubo_driver/server_host", (serverHostParameter=="")? "192.168.88.15" : serverHostParameter);

        robot_driver.run();

        ros::Duration(1).sleep(); // Time needed to initialize variables

        //InitializeDriver(robot_driver); // Just sets initial position

		ros::Duration(2).sleep(); // Time needed to finish AUBO's first motion

		set_record = true;
        timinglog.setRecording(set_record);

		ros::Rate loop_rate(frequency);
		
		dt = 1/frequency;

		// Control in z-direction only
		Kpf << 	0*-1./600, 	0, 			0,
				0, 			0*-1./600, 	0,
				0, 			0, 			-1./600;

		// force control integrator initial value (initial cartesian position)
		joint_pos_initial = getJointAngle(robot_driver);
		joint_pos_target = joint_pos_initial; // 'OFFLINE' TRAJ
		kinematics(joint_pos_initial);
		force_control_integrator_output = p;

		while (ros::ok())
		{
			//joint_pos_initial = getJointAngle(robot_driver);
			joint_pos = getJointAngle(robot_driver);
			//std::cout << "## joint_pos = " << joint_pos << std::endl;
			
			ros::spinOnce(); // force callback called (set force)
			//std::cout << "## force = " << force << std::endl;

			kinematics(joint_pos_target); // set p (end-effector cartesian position) and q (end-effector quaternion)
			//std::cout << "## p = " << p << std::endl;

			force_control(); // input = force (set by force callback), force_ref (set at initialization) 
			// output = pd ()
			//std::cout << "## pd = " << pd.transpose() << std::endl;
				
			control(); // input = pd, p / output = delta_joint_pos	
			//std::cout << "## delta_joint_pos = " << delta_joint_pos << std::endl;		

            estimative_update(loop_rate);

            loop_rate.sleep();
			sendJointPosOnline(robot_driver);
			//std::cout << "## joint_pos_target = " << joint_pos_target << std::endl;
			
			if (set_record)
            	record_addtoLog();
		}
	}
    */
};


#endif /* AUBO_DRIVER_H_ */