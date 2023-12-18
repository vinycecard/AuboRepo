#include "ros/ros.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Bool.h>
#include <cmath>

#include <Eigen/Dense>

#include "aubo_driver/aubo_driver.h"
#include "aubo_driver.cpp"
#include "logger/log.h"

#define pi M_PI
#define MAX_JOINT_ACC 100.0/180.0*M_PI  // unit rad/s^2
#define MAX_JOINT_VEL 100.0/180.0*M_PI   // unit rad/s
#define MAX_END_ACC    4                // unit m/s^2
#define MAX_END_VEL    2                // unit m/s

/*
	Based on aubo_driver/src/testAuboAPI.cpp and trajectory/src/joint_control.cpp
*/

class JointControl
{
private:
    int square_direction, count;
    double t, wp, frequency, dt, amplitude, left_time, t0, t_new, interval, last_t, tau, t_position_sent;
	double cycletime_avg = 0;
    bool first_pub;

    double position_sent[ARM_DOF], jointAngle[ARM_DOF];

    Eigen::VectorXd ui, u, ud, udd, theta, last_theta, vel_f, ui_teo, u_teo, joint_reference;

    ros::NodeHandle nh;

	TimingLog timinglog;
    
	ros::Publisher joint_cmd_Pub, record_Pub;
    ros::Subscriber record_Sub;
    ros::Time pub_start, start_time;
    trajectory_msgs::JointTrajectoryPoint waypoint;
	std_msgs::Bool set_record;

    double zero_position[ARM_DOF] = {0};
    double initial_position[ARM_DOF] = {0.0/180.0*pi,  -7.2917/180.0*pi,  -75.6946/180.0*pi, 21.5967/180.0*pi, -90.0/180.0*pi, 0.0/180.0*pi};
	double initial_position2[ARM_DOF] = {0.0/180.0*pi,  -7.2917/180.0*pi,  -75.6946/180.0*pi, (21.5967-90.0)/180.0*pi, -90.0/180.0*pi, 0.0/180.0*pi};

    aubo_robot_namespace::wayPoint_S wayPoint_S;
    aubo_robot_namespace::JointParam jointParam;

	//-------------------------------------- Reference ---------------------------------------------------

	double wo, trace_R, trace_dR, g, dg, theta_d, dtheta_d;

	Eigen::VectorXd pd, vd, qd, dqd, wd, f, df, u_d, du_d;
	Eigen::MatrixXd R, dR, E;

	//-------------------------------------- Kinematics -------------------------------------------------------

	// Nico: Aubo
	double a2, a3, d1, d2, d5, d6, ti, alpha_i, ai, di;
	double cai, sai, cai2, sai2;

	Eigen::VectorXd p, q, zq, pqi, pqi_star, pqi_star_temp, p_J, h, h_temp, qi, q_temp;
	Eigen::MatrixXd DH, P, H, J, I, I6;

	//-------------------------------------- Kinematic Control -------------------------------------------

	double Kp, Ko;

	Eigen::VectorXd pe, qe, u_ws, theta_min, theta_max, u_max, joint_error;

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

	Eigen::VectorXd rotateQuat(const Eigen::VectorXd& position, const Eigen::VectorXd& quaternion) // rotate position vector using quaternion
	{
		Eigen::VectorXd new_position(3);
		Eigen::VectorXd position_aux(4), new_position_aux1(4), new_position_aux2(4);

		position_aux << 0, position(0), position(1), position(2);

		new_position_aux1 = qdot(position_aux, conjugate(quaternion));
		new_position_aux2 = qdot(quaternion, new_position_aux1);

		new_position = new_position_aux2.tail(3); // new_position = q*position*q^-1

		return new_position;
	}

public:
    JointControl(int argc_, char** argv_): first_pub(true), amplitude(0.5), wp(0.8), count(1),
	frequency(130.0), tau(.5), a2(.408), a3(.376), d1(.0985), d2(.1215), d5(.1025), d6(.094),
	timinglog("/AUBO/logs/")
    {
        joint_cmd_Pub = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/joint_cmd",2000);
		record_Pub = nh.advertise<std_msgs::Bool>("/timinglog_record",5);
		record_Sub = nh.subscribe<std_msgs::Bool>("/timinglog_record", 5, &JointControl::RecordCallback, this);
		
        ui = Eigen::VectorXd::Zero(ARM_DOF);
        ui_teo = Eigen::VectorXd::Zero(ARM_DOF);
        u = Eigen::VectorXd::Zero(ARM_DOF);
        u_teo = Eigen::VectorXd::Zero(ARM_DOF);
        ud = Eigen::VectorXd::Zero(ARM_DOF);
        udd = Eigen::VectorXd::Zero(ARM_DOF);
		joint_reference = Eigen::VectorXd::Zero(ARM_DOF);
		joint_error = Eigen::VectorXd::Zero(ARM_DOF);

        theta = Eigen::VectorXd::Zero(ARM_DOF);
        last_theta = Eigen::VectorXd::Zero(ARM_DOF);

        vel_f = Eigen::VectorXd::Zero(ARM_DOF);

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

		DH.resize(ARM_DOF,4);
		P.resize(3,ARM_DOF);
		H.resize(3,ARM_DOF);
		J.resize(6,ARM_DOF);

		pe.resize(3);
		qe.resize(4);
		u_ws.resize(6);
		theta_min.resize(ARM_DOF);
		theta_max.resize(ARM_DOF);
		u_max.resize(ARM_DOF);

        waypoint.positions.resize(ARM_DOF);
		waypoint.velocities.resize(ARM_DOF);
		waypoint.accelerations.resize(ARM_DOF);

        square_direction = 1;

		for (int i = 0; i < ARM_DOF; i++)
			joint_reference(i) = initial_position2[i];

		joint_reference(2) += pi/2;

        Kp = .5;
        Ko = 0;

        zq.resize(4);
        DH.resize(ARM_DOF, 8);

    	// Denavit-Hartenberg Modified matrix (lengths in [m], angles in [rad]).
		//cai = cos(alpha(i-1)), cai2 = cos(alpha(i-1)/2);
		//sai = sin(alpha(i-1)), sai2 = sin(alpha(i-1)/2);

		// a_(i-1), alpha_(i-1), d_i, theta_i, 		cai, 	sai,		cai2,			sai2,
		DH <<  	0, 		0, 		d1,		pi,			1, 		0,			1, 				0,
				0, 		-pi/2,	d2,		-pi/2,  	0,		-1,			cos(-pi/4), 	sin(-pi/4),
				a2, 	pi, 	0,  	0,			-1,		0,			0,				1,
				a3,		pi,  	0,  	-pi/2,		-1,		0,			0,				1,
				0,  	-pi/2, 	d5,   	0,			0,		-1,			cos(-pi/4),		sin(-pi/4),
				0,  	pi/2,  	d6,     0,			0,		1,			cos(pi/4),		sin(pi/4);

		zq << 0, 0, 0, 1; // zq = [0; z]

		// Limits
		theta_min << -3.054, -3.054, -3.054, -3.054, -3.054, -3.054; //rad (-175 degrees)
		theta_max << 3.054, 3.054, 3.054, 3.054, 3.054, 3.054;      //rad (175 degrees)
		//u_max << 2.618, 2.618, 2.618, pi, pi, pi;                //rad/s (150 / 180 degrees)

		I = Eigen::MatrixXd::Identity(3,3);
		I6 = Eigen::MatrixXd::Identity(6,6);
    }

    ~JointControl()
	{   
		std::cout << std::endl << "JointControl Object Destroyed" << std::endl << std::endl;

		std::cout << "cycletime average = " << cycletime_avg << " secs = ";
		std::cout << 1/cycletime_avg << " hz" << std::endl;
	}

	void RecordCallback(const std_msgs::Bool::ConstPtr &msg)
	{
		timinglog.setRecording(msg->data);
	}

    void InitializeDriver(aubo_driver::AuboDriver &robot_driver)
    {
        int ret;
        ret = robot_driver.robot_send_service_.robotServiceLeaveTcp2CanbusMode();
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to leave Tcp2Canbus Mode, error code:%d", ret);
    
        /** Initialize move properties ***/
        ret = robot_driver.robot_send_service_.robotServiceInitGlobalMoveProfile();
		if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to Init Global Move Profile, error code:%d", ret);

        /** Set Max joint acc and vel***/
        aubo_robot_namespace::JointVelcAccParam jointMaxAcc;
        aubo_robot_namespace::JointVelcAccParam jointMaxVelc;
        for(int i = 0; i < ARM_DOF; i++)
        {
            jointMaxAcc.jointPara[i] = MAX_JOINT_ACC;
            jointMaxVelc.jointPara[i] = MAX_JOINT_VEL;
        }
        robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxAcc(jointMaxAcc);
        robot_driver.robot_send_service_.robotServiceSetGlobalMoveJointMaxVelc(jointMaxVelc);

        /** Robot move to zero position **/
        ret = robot_driver.robot_send_service_.robotServiceJointMove(initial_position2, true);
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to move to initial position, error code:%d", ret);

        ros::Duration(.5).sleep();

        ret = robot_driver.robot_send_service_.robotServiceEnterTcp2CanbusMode();
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to enter Tcp2Canbus Mode, error code:%d", ret);         
    }

	void getJointAngle(aubo_driver::AuboDriver &robot_driver)
    {
        robot_driver.robot_send_service_.robotServiceGetJointAngleInfo(jointParam);

        for (int i = 0; i < ARM_DOF; i++)
			theta(i) = jointParam.jointPos[i];

		ui = theta;
		t = (ros::Time::now() - start_time).toSec();
    }

	void reference()
	{	
		// (XZ plane) (Circle centered at Init2 Position)
		// End-effector position reference (m). 
		pd(0) = .2*sin(wp*t) - .3918;
		pd(1) = 0 - .1215;
		pd(2) = .1*cos(wp*t)*0 + .7440; // = 0, entao senoide em x

		// End-effector velocity reference (m/s).
		vd(0) = .2*wp*cos(wp*t);
		vd(1) = 0;
		vd(2) = -.1*wp*sin(wp*t)*0;

		/* 
        // (Zero Position)
		// End-effector orientation reference (rotation matrix).
		R(0,0) = 1;  R(0,1) = 0;  R(0,2) = 0;
		R(1,0) = 0;  R(1,1) = 0;  R(1,2) = -1;
		R(2,0) = 0;  R(2,1) = 1;  R(2,2) = 0;
		*/
		/*
		// (Init Position)
		// End-effector orientation reference (rotation matrix).
		R(0,0) = 0;  R(0,1) = -1;  R(0,2) = 0;
		R(1,0) = -1;  R(1,1) = 0;  R(1,2) = 0;
		R(2,0) = 0;  R(2,1) = 0;  R(2,2) = -1;
		*/
        // (Init2 Position)
		// End-effector orientation reference (rotation matrix).
		R(0,0) = 0;  R(0,1) = 0;  R(0,2) = -1;
		R(1,0) = -1;  R(1,1) = 0;  R(1,2) = 0;
		R(2,0) = 0;  R(2,1) = 1;  R(2,2) = 0;

		// End-effector orientation derivative reference (rotation matrix).
		dR(0,0) = 0;  dR(0,1) = 0;  dR(0,2) = 0;
		dR(1,0) = 0;  dR(1,1) = 0;  dR(1,2) = 0;
		dR(2,0) = 0;  dR(2,1) = 0;  dR(2,2) = 0;
/*
		// ------------- Rotation matrix - Quaternion Conversion --------------------
		trace_R = R.trace();
		trace_dR = dR.trace(); // = 0

		f(0) = trace_R + 1;
		f(1) = R(2,1) - R(1,2);
		f(2) = R(0,2) - R(2,0);
		f(3) = R(1,0) - R(0,1);

		df(0) = trace_dR;			// df = 0
		df(1) = dR(2,1) - dR(1,2);
		df(2) = dR(0,2) - dR(2,0);
		df(3) = dR(1,0) - dR(0,1);

		g = sqrt(trace_R + 1);
		dg = trace_dR/(2.*g); // = 0

		qd = f/(2.*g);
		dqd = (df*g-f*dg)/(2.*pow(g,2)); // =0


		// Quaternion Jacobian
		E.row(0) = -qd.tail(3).transpose(); 
		E.block(1,0,3,3) = qd(0)*I - crossmat(qd.tail(3));

		wd = 2*E.transpose()*dqd;
*/
		wd << 0, 0, 0;
	}

	void kinematics()
	{  
		q << 1, 0, 0, 0; // End-Effector quaternion.
		p << 0, 0, 0;		 // End-Effector position (cm).

		P = Eigen::MatrixXd::Constant(3,6,0);
		H = P;

		for (int i = 0; i < ARM_DOF; i++)
		{
		  // DH Modified
		  ti = theta(i) + DH(i,3);
		  alpha_i = DH(i,1);
		  ai = DH(i,0);
		  di = DH(i,2);

		  double cti2 = cos(ti/2);
		  double sti2 = sin(ti/2);
		  
		  double cai = DH(i,4);
		  double sai = DH(i,5);
		  double cai2 = DH(i,6);
		  double sai2 = DH(i,7);
		
		  pqi(0) = 0;                       // [0; trans homog position]
		  pqi(1) = ai;
		  pqi(2) = -di*sai;
		  pqi(3) = di*cai;


	    pqi_star_temp = qdot(pqi,conjugate(q));
	    pqi_star = qdot(q,pqi_star_temp);       // pqi_star = q*position*q^-1

	    P.col(i) = pqi_star.tail(3);
	    p += P.col(i);

		qi(0) = cti2*cai2;
		qi(1) = cti2*sai2;
		qi(2) = -sti2*sai2;
		qi(3) = sti2*cai2;

		q_temp = qdot(q,qi);
		q = q_temp;

		h_temp = qdot(zq,conjugate(q));
		h = qdot(q,h_temp);

		H.col(i) = h.tail(3);
		}

		p_J = p;

        // matrix.block(i,j,p,q) = Block of size (p,q), starting at (i,j)
        // bottom 3 rows defined
		J.block(3,0,3,6) = H; // End-Effector Geometric Orientation Jacobian.

		for (int i = 0; i < ARM_DOF; i++)
		{
			p_J -= P.col(i);
		  	J.block(0,i,3,1) = crossmat(H.col(i))*p_J; // End-Effector Geometric Position Jacobian.
		}
	}

    void calc_fk(aubo_driver::AuboDriver &robot_driver)
    {
        int ret;
        Eigen::VectorXd::Map(&jointAngle[0], theta.size()) = theta;
        ret = robot_driver.robot_send_service_.robotServiceRobotFk(jointAngle, ARM_DOF, wayPoint_S);
		// return position as wayPoint_S 
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to calculate forward kinematic, error code:%d", ret);
    }

	void control() // Cartesian position control using inverse jacobian
	{
		//-------------------------- Kinematic Control Law -------------------------------------------

		// Errors.
		pe = pd - p;
		qe = qdot(qd,conjugate(q));

		// Workspace Control Signal.
		u_ws.head(3) = vd + Kp*pe;
		u_ws.tail(3) = wd + Ko*qe.tail(3);

	  // Joint Space Control Signal.
        if (!first_pub) 
       	{
		    u = J.inverse()*u_ws;				// IF FIRSTPUB NECESSARY?
        }

		ui += u*dt;
	}

	void controljoint() // joint positial control (proportional)
	{
		joint_error = joint_reference - theta;

		u = Kp*joint_error;

		ui += u*dt;
	}

    void sinusoid() // sets u as sinusoid
    {
        if (first_pub) 
        {
		    start_time = ros::Time::now();
            ui = theta;
            ui_teo = theta;
            t0 = asin(theta(ARM_DOF-1)/amplitude)/wp;
            last_t = 0;
        }

    
        t = (ros::Time::now() - start_time).toSec();
        t_new = t + t0;

            ui_teo(ARM_DOF-1) = amplitude*sin(wp*t_new);
            u_teo(ARM_DOF-1) = amplitude*wp*cos(wp*t_new);

	        //ui(ARM_DOF-1) = amplitude*sin(wp*t_new);
	        u(ARM_DOF-1) = amplitude*wp*cos(wp*t_new);
	        //ud(ARM_DOF-1) = -amplitude*wp*wp*sin(wp*t_new);
    }

    void squarewave() // TO DO
    {
        if (first_pub) 
	        start_time = ros::Time::now();

	    t = (ros::Time::now() - start_time).toSec();
  
        if (fmod(t,10) < 5)
        {
            square_direction = 1;
            ui(ARM_DOF-1) = wp*fmod(t,10)*square_direction;
        }
        else
        { 
            square_direction = -1;
            ui(ARM_DOF-1) = wp*(10-fmod(t,10))*-square_direction;
        }

	    //ui(ARM_DOF-1) = amplitude*fmod(t,10)*square_direction;
	    u(ARM_DOF-1) = amplitude*square_direction;
	    //ud(ARM_DOF-1) = -amplitude*wp*wp*sin(wp*t)*0;
        //udd(ARM_DOF-1) = -amplitude*wp*wp*wp*cos(wp*t)*0;
    }

	void checklimits() // Check Joint limit (|3.054| rad = |175| deg) and Check deltaJointPosition limit (|0.01| rad)
	{
		for (int i = 0; i < ARM_DOF; i++)
		{
			if (((theta(i) >= .95*theta_max(i)) && (ui(i) > 0)) || ((theta(i) <= .95*theta_min(i)) && (ui(i) < 0)))
		    	ui(i) = theta(i);
			else if (ui(i) - theta(i) > 0.01) // Check deltaJointPosition limit (0.01 rad)
		  		ui(i) = theta(i) + 0.01;
			else if (ui(i) - theta(i) < -0.01) // Check deltaJointPosition limit (-0.01 rad)
		  		ui(i) = theta(i) - 0.01;
		}
	}

    void velocity_estimative()
	{
		if (!first_pub)
		{	
			interval = t - last_t;

			// First-order discrete filter (Tustin Method)
			vel_f = (2*(theta - last_theta) - (interval - 2*tau)*vel_f)/(interval + 2.*tau);
		}
	}

    void sendJointPos(aubo_driver::AuboDriver &robot_driver)
    {   
        int ret;

        memcpy(position_sent, ui.data(), sizeof(double) * ARM_DOF);

		ret = robot_driver.robot_send_service_.robotServiceJointMove(position_sent, true);
        //ret = robot_driver.robot_send_service_.robotServiceSetRobotPosData2Canbus(position_sent); 
        if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to move to position_sent, error code:%d", ret);
    }

    void pub_traj_msg() // publishes joint angle (ui) sent to AUBO through /joint_cmd topic
    { 
        if (first_pub)
	    {
			set_record.data = false;
		    record_Pub.publish(set_record);
			first_pub = false;
		    pub_start = ros::Time::now();
	    }	

        // Conversion from Eigen vector to std vector
        Eigen::VectorXd::Map(&waypoint.positions[0], ui.size()) = ui;
        Eigen::VectorXd::Map(&waypoint.velocities[0], u.size()) = u;
        Eigen::VectorXd::Map(&waypoint.accelerations[0], theta.size()) = u_teo;

        waypoint.time_from_start = ros::Time::now() - pub_start;

        joint_cmd_Pub.publish(waypoint);

		
		//for (unsigned int i = 0; i < ARM_DOF; i++)
    	//{
			int i = 2;
        	timinglog.addToLog(std::to_string(joint_error(i)), "error_Joint" + std::to_string(i+1));
			timinglog.addToLog(std::to_string(theta(i)), "pos_Joint" + std::to_string(i+1));
    	//}
		
    }

    void estimative_update(ros::Rate &loop_rate)
	{
		last_theta = theta;
		last_t = t;
		cycletime_avg = (cycletime_avg*(count-1) + loop_rate.cycleTime().toSec())/count;
		count++;		
	}

    void execute()
	{   
		int ret;

        aubo_driver::AuboDriver robot_driver;

        ros::param::set("/aubo_driver/server_host", "192.168.88.25");

        robot_driver.run();

        //If connect to a real robot, then you need initialize the dynamics parameters
        aubo_robot_namespace::ROBOT_SERVICE_STATE result;

        //tool parameters
        aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
        memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

        ret = robot_driver.robot_send_service_.rootServiceRobotStartup(toolDynamicsParam,6, true, true, 1000, result);
		if(ret != aubo_robot_namespace::InterfaceCallSuccCode)
            ROS_ERROR("Failed to Startup Robot, error code:%d", ret);

        ros::Duration(1).sleep(); // Time needed to initialize variables

        InitializeDriver(robot_driver);

		ros::Rate loop_rate(frequency);
		
		dt = loop_rate.expectedCycleTime().toSec();

		while (ros::ok())
		{
            getJointAngle(robot_driver);

			kinematics();
			reference();

			//control();
			controljoint();

			//sinusoid();

            //velocity_estimative();

			checklimits();
		
            //sendJointPos(robot_driver);			

            pub_traj_msg();
            // first_pub = false;

            estimative_update(loop_rate);

            loop_rate.sleep();
		}
	}

};


int main(int argc, char **argv)
{   
    ros::init(argc, argv, "control_Nico");
	
	JointControl joint_control(argc, argv);

    joint_control.execute();

    return 0;
}