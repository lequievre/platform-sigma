/*
 *  Kamal mohy el dine 
 *  Institut Pascal UMR6602
 *  kamal.mohy.el.dine@gmail.com
 * 
*/

#include <torque_based_position_gazebo.h>
#include "math.h"
// For plugin
#include <pluginlib/class_list_macros.h>

#include <algorithm>
#include <Eigen/Dense>
#include <utils/pseudo_inversion.h>

namespace kuka_lwr_controllers 
{
    TorqueBasedPositionControllerGazebo::TorqueBasedPositionControllerGazebo() {}
    TorqueBasedPositionControllerGazebo::~TorqueBasedPositionControllerGazebo() {}

   bool TorqueBasedPositionControllerGazebo::init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n)
    {
		
		robot_namespace_ = n.getNamespace();
		
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start init of robot %s !",robot_namespace_.c_str());
		#endif
			
        if( !(KinematicChainControllerBase<hardware_interface::KUKAJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("TorqueBasedPositionController: Couldn't initilize TorqueBasedPositionController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        //initializes the solvers
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        id_solver_.reset(new KDL::ChainDynParam( kdl_chain_, gravity_));
		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
		fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));
		
		
        sub_command_ 		= n.subscribe("command"				,1, &TorqueBasedPositionControllerGazebo::commandCB			, this);
        sub_kp_cartesian_ 	= n.subscribe("setcartesianKp"		,1, &TorqueBasedPositionControllerGazebo::setcartesianKp	, this);
        sub_kd_cartesian_ 	= n.subscribe("setcartesianKd"		,1, &TorqueBasedPositionControllerGazebo::setcartesianKd	, this);
		sub_traj_			= n.subscribe("/traj_cmd"			,1, &TorqueBasedPositionControllerGazebo::TrajPathPointCB	, this);
		sub_ft_ 			= n.subscribe("sensor_readings"		,1, &TorqueBasedPositionControllerGazebo::ft_readingsCB		, this);
		sub_kp_joints_  	= n.subscribe("setjointsKp"			,1, &TorqueBasedPositionControllerGazebo::setjointsKp		, this); 
		sub_kd_joints_  	= n.subscribe("setjointsKd"			,1, &TorqueBasedPositionControllerGazebo::setjointsKd		, this); 
		sub_kp_force_		= n.subscribe("setforceKp"			,1, &TorqueBasedPositionControllerGazebo::setforceKp		, this); 
		sub_ee_pos_			= n.subscribe("/aruco_single/pose"	,1, &TorqueBasedPositionControllerGazebo::setRollPitchYaw	, this); 
		sub_stiffness_damping_ = n.subscribe("setStiffnessDamping"		,1, &TorqueBasedPositionControllerGazebo::setStiffnessDamping	, this); 

		pub_traj_resp_		= n.advertise<kuka_lwr_controllers::TrajPathPoint>("traj_resp", 1000);
		pub_tau_cmd_		= n.advertise<std_msgs::Float64MultiArray>("tau_cmd", 1000);
		pub_F_des_			= n.advertise<geometry_msgs::WrenchStamped>("F_des", 1000);
		pub_Sv				= n.advertise<std_msgs::Float64>("Sv", 1000);
		pub_Sf 				= n.advertise<std_msgs::Float64>("Sf", 1000);
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
		d_wall_ = 0.5;
	

		//resizing the used vectors
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());
		Jkdl_.resize(kdl_chain_.getNrOfJoints());
		Jnt_vel_.resize(kdl_chain_.getNrOfJoints());
		//Creating Jnt_vel_ object of type JntArrayVel as attribute for the fk_vel_solver_->JntToCart(Jnt_vel_, V_current_);
		Jnt_vel_.q=joint_msr_states_.q;
		Jnt_vel_.qdot=joint_msr_states_.qdot;
		
		Kp_joints_.resize(kdl_chain_.getNrOfJoints());
		Kd_joints_.resize(kdl_chain_.getNrOfJoints());
		stiff_.resize(kdl_chain_.getNrOfJoints());
		damp_.resize(kdl_chain_.getNrOfJoints());

		
		for (std::size_t i=0; i<joint_handles_.size()-3; i++)
		{
			Kp_joints_(i) = 200.0;
			Kd_joints_(i) = 50.0;
		}
		for (std::size_t i=4; i<joint_handles_.size(); i++)
		{
			Kp_joints_(i) = 50.0;
			Kd_joints_(i) = 10.0;
		}
		for (std::size_t i=0; i<joint_handles_.size()-3; i++)
		{
			stiff_(i) = 1000.0;
			damp_(i) = 1.0;
		}
		for (std::size_t i=4; i<joint_handles_.size(); i++)
		{
			stiff_(i) = 50.0;
			damp_(i) = 0.3;
		}
		Kp_cartesian_ .resize(6);
		Kd_cartesian_ .resize(6);
		
		KDv_ 		= MatrixXd::Zero(6,6);
		KPv_ 		= MatrixXd::Zero(6,6);
		KPf_ 		= MatrixXd::Zero(6,6);
		
 		for (std::size_t i=0; i<3; i++)
		{
			Kp_cartesian_(i) = 100.0;
			Kd_cartesian_(i) = 50.0;
			
			KPv_(i,i )=Kp_cartesian_(i);
			KDv_(i,i )=Kd_cartesian_(i);
		}    
		KPf_(0,0) = 100;
		
		S_v_    	= MatrixXd::Identity(6,6);				// selection matrix for position controlled direction
		
		S_v_.bottomRightCorner(3,3) = MatrixXd::Zero(3, 3);
		S_f_    	= MatrixXd::Zero(6,6);					// selection matrix for velocity controlled direction
		
		traj_des_.resize(6);
		SetToZero (traj_des_);

		q_des_.resize(kdl_chain_.getNrOfJoints());
		SetToZero (q_des_);
		

	
		LAMDA_.resize(6,6);
		Alpha_v_ 	= MatrixXd::Zero(6, 1);
		FT_sensor_	= VectorXd::Zero(6);
		F_des_		= VectorXd::Zero(6);
		Tau_cmd_	= VectorXd::Zero(6);
		
		
		//for (std::size_t i=0; i<3; i++)
		//F_des_(i) = 50;
		F_des_(0) = 70;
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: End of Start init of robot %s !",robot_namespace_.c_str());
		#endif

		return true;
	}
	
	void TorqueBasedPositionControllerGazebo::starting(const ros::Time& time)
    {
        
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
		count = 0;
    }
    
    void TorqueBasedPositionControllerGazebo::stopping(const ros::Time& time)
	{
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
		
	}
	
    void TorqueBasedPositionControllerGazebo::update(const ros::Time& time, const ros::Duration& period)
    {	
		ros::Time begin = ros::Time::now();
		/****** messages to be published for analyses *****/
		kuka_lwr_controllers::TrajPathPoint traj_resp_msg;
		std_msgs::Float64MultiArray tau_cmd_msg;	 tau_cmd_msg.data.resize(7);	
		std_msgs::Float64 Sv_msg, Sf_msg;
        geometry_msgs::WrenchStamped  F_des_msg;
        	
		double torque;
		
		if (cmd_flag_)
		{

			// update the commanded position to the actual, so that the robot doesn't 
			// go back at full speed to the last commanded position when the stiffness 
			// is raised again
			for(size_t i=0; i<joint_handles_.size(); i++) 
			{
				joint_msr_states_.q(i) = joint_handles_[i].getPosition();
				joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();		
			}
			// position control
			/*for(size_t i=0; i<joint_handles_.size(); i++) 
			{
			
			torque = (Kp_(i)*(q_des_(i)-joint_handles_[i].getPosition()))+(Kd_(i)*(-joint_handles_[i].getVelocity())) +G_.data(i);
			joint_handles_[i].setCommand(torque); // Set a value of redundant joint.
			}*/
			
			
			 /*for(size_t i=0; i<joint_handles_.size(); i++) 
			{
				joint_handles_[i].setCommandPosition(joint_handles_[i].getPosition());
				//joint_handles_[i].setCommandTorque(0.0); // Set a value of torque to 0.0 for each joint.
				joint_handles_[i].setCommandStiffness(stiff_(i));
				joint_handles_[i].setCommandDamping(damp_(i));
			}*/
				joint_handles_[0].setCommandPosition(joint_handles_[0].getPosition());
				joint_handles_[0].setCommandStiffness(stiff_(0));
				joint_handles_[0].setCommandDamping(damp_(0));
				
				joint_handles_[1].setCommandPosition(joint_handles_[1].getPosition());
				joint_handles_[1].setCommandStiffness(stiff_(1));
				joint_handles_[1].setCommandDamping(damp_(1));
				
				joint_handles_[3].setCommandPosition(joint_handles_[3].getPosition());
				joint_handles_[3].setCommandStiffness(stiff_(3));
				joint_handles_[3].setCommandDamping(damp_(3));
			//-------------------------------------------------------------------------------// 
			//-------------------------------------computing by solvers----------------------//   
			//-------------------------------------------------------------------------------// 
			
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, Jkdl_);						// computing Jacobian with KDL
            id_solver_->JntToMass(joint_msr_states_.q, M_);									// computing Inertia matrix		
			id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);		// computing Coriolis torques	
			id_solver_->JntToGravity(joint_msr_states_.q, G_);								// computing Gravity torques
		
			//fcn->JntToCart(const JntArray& q_in, Frame& p_out, int seg_nr) 
            fk_pos_solver_->JntToCart(joint_msr_states_.q, P_current_); 					// computing forward kinematics
           
            //fcn->JntToCart(const JntArrayVel& q_in,FrameVel& out,int segmentNr=-1);
            Jnt_vel_.q=joint_msr_states_.q;													
			Jnt_vel_.qdot=joint_msr_states_.qdot;
            fk_vel_solver_->JntToCart(Jnt_vel_, V_current_);								// computing velocities
			
			
			//ROS_INFO("***** joint velocity **************");
			//ROS_INFO("q%d_dot=%lf",0+1,joint_handles_[0].getVelocity());
			//ROS_INFO("***** joint position **************");
			//ROS_INFO("q5=%lf q6=%lf q7=%lf",joint_msr_states_.q(4),joint_msr_states_.q(5),joint_msr_states_.q(6));
			//ROS_INFO("***** Current velocity **************");
			//ROS_INFO("vx=%lf vy=%lf vz=%lf ",V_current_.GetTwist ().vel.x(),V_current_.GetTwist ().vel.y(),V_current_.GetTwist ().vel.z());
			        
            //------------------------------------------------------------------------------------------------------------------------------------// 
            //-------------------------------------------Hybrid Control Calculation---------------------------------------------------------------//   
            //------------------------------------------------------------------------------------------------------------------------------------//     
            
            J6x6_ =   Jkdl_.data;        // save the KDL jacoian to the 6x6 one to modify it
            removeColumn(J6x6_, 2);      // removing the column corresponding to the redundant joint and resising the matrix
            J_trans_= J6x6_.transpose();
            //J_inv_  = J6x6_.inverse();
            //J_inv_trans_ = J_inv_.transpose();
           
            I6x6_= M_.data;             // save the KDL Inertia matrix to the 6x6 one to modify it
			removeColumn(I6x6_, 2);     // removing the column corresponding to the redundant joint and resising the matrix
			removeRow(I6x6_, 2);        // removing the row corresponding to the redundant joint and resising the matrix
			
			
			//calculatePsuedoInertia(J6x6_, I6x6_, LAMDA_ );
			calculatePsuedoInertia(Jkdl_.data, M_.data, LAMDA_ );	
			//LAMDA_ = ((Jkdl_.data)*(M_.data.inverse())*(Jkdl_.data.transpose())).inverse();				// calculate inertia matrix in workspace
		    calculateAccelerationCommand(P_current_, V_current_,traj_des_,Alpha_v_ );						// calculate the acceleration command 
		    SelectionMatricesTransitionControl();                                                           // change the selection matrices based on the distance to wall from camera
		    calculateForceCommand(FT_sensor_, F_des_, F_cmd_);
			calculateJointTorques(J6x6_,Alpha_v_, Tau_cmd_);												// calculate the joint torques
			
			
			tau_cmd_msg.data[0] = Tau_cmd_(0)+C_(0)+G_(0);
			tau_cmd_msg.data[1] = Tau_cmd_(1)+C_(1)+G_(1);
			tau_cmd_msg.data[2] = (Kp_joints_(2)*(0-joint_handles_[2].getPosition()))+(Kd_joints_(2)*(-joint_handles_[2].getVelocity()))+C_(2)+G_(2);
			tau_cmd_msg.data[3] = Tau_cmd_(2)+C_(3)+G_(3);
			/*tau_cmd_msg.data[4] = (Kp_joints_(4)*(0-joint_handles_[4].getPosition()))+(Kd_joints_(4)*(-joint_handles_[4].getVelocity()))+C_(4)+G_(4);
			tau_cmd_msg.data[5] = ((Kp_joints_(5)*((-joint_handles_[1].getPosition()+joint_handles_[3].getPosition())- joint_handles_[5].getPosition()
								))+(Kd_joints_(5)*((-joint_handles_[1].getVelocity()+joint_handles_[3].getVelocity())-joint_handles_[5].getVelocity())))+C_(5)+G_(4);
			tau_cmd_msg.data[6] = (Kp_joints_(6)*(0-joint_handles_[6].getPosition()))+(Kd_joints_(6)*(-joint_handles_[6].getVelocity()))+C_(6)+G_(6);
			*/
			
			joint_handles_[2].setCommandPosition(0.0);
			joint_handles_[2].setCommandStiffness(stiff_(2));
			joint_handles_[2].setCommandDamping(damp_(2));
			
			joint_handles_[4].setCommandPosition(0.0);
			joint_handles_[4].setCommandStiffness(stiff_(4));
			joint_handles_[4].setCommandDamping(damp_(4));
			
			joint_handles_[5].setCommandPosition(-joint_handles_[1].getPosition()+joint_handles_[3].getPosition());
			joint_handles_[5].setCommandStiffness(stiff_(5));
			joint_handles_[5].setCommandDamping(damp_(5));
			
			joint_handles_[6].setCommandPosition(-joint_handles_[0].getPosition());
			joint_handles_[6].setCommandStiffness(stiff_(6));
			joint_handles_[6].setCommandDamping(damp_(6));
			
			joint_handles_[0].setCommandTorque(tau_cmd_msg.data[0]) ;
			joint_handles_[1].setCommandTorque(tau_cmd_msg.data[1]);
			joint_handles_[2].setCommandTorque(tau_cmd_msg.data[2]*0.0); // Set a value of redundant joint.
			joint_handles_[3].setCommandTorque(tau_cmd_msg.data[3]);
			joint_handles_[4].setCommandTorque(0.0);
			joint_handles_[5].setCommandTorque(0.0);
			joint_handles_[6].setCommandTorque(0.0);
			
			/*joint_handles_[4].setCommandTorque(tau_cmd_msg.data[4]);
			joint_handles_[5].setCommandTorque(tau_cmd_msg.data[5]);
			joint_handles_[6].setCommandTorque(tau_cmd_msg.data[6]);*/
			
			/*	
	        joint_handles_[0].setCommandTorque(Tau_cmd_(0)) ;
			joint_handles_[1].setCommandTorque(Tau_cmd_(1));
			torque = (Kp_joints_(2)*(0-joint_handles_[2].getPosition()))+(Kd_joints_(2)*(-joint_handles_[2].getVelocity()));
			joint_handles_[2].setCommandTorque(torque); // Set a value of redundant joint.
			joint_handles_[3].setCommandTorque(Tau_cmd_(2));
			joint_handles_[4].setCommandTorque((Kp_joints_(4)*(0-joint_handles_[4].getPosition()))+(Kd_joints_(4)*(-joint_handles_[4].getVelocity())));
			joint_handles_[5].setCommandTorque((Kp_joints_(5)*((-joint_handles_[1].getPosition()+joint_handles_[3].getPosition())- joint_handles_[5].getPosition()
											))+(Kd_joints_(5)*((-joint_handles_[1].getVelocity()+joint_handles_[3].getVelocity())-joint_handles_[5].getVelocity())));
			joint_handles_[6].setCommandTorque((Kp_joints_(6)*(0-joint_handles_[6].getPosition()))+(Kd_joints_(6)*(-joint_handles_[6].getVelocity())));
			*/
			count++;
			// publishing data 
				traj_resp_msg.pos.x = P_current_.p.x();
				traj_resp_msg.pos.y = P_current_.p.y();
				traj_resp_msg.pos.z = P_current_.p.z();
				traj_resp_msg.vel.x = V_current_.GetTwist ().vel.x();
				traj_resp_msg.vel.y = V_current_.GetTwist ().vel.y();
				traj_resp_msg.vel.z = V_current_.GetTwist ().vel.z();
				traj_resp_msg.acc.x = 0.0;
				traj_resp_msg.acc.y = 0.0;
				traj_resp_msg.acc.z = 0.0;
				pub_traj_resp_.publish(traj_resp_msg);
				
				/*tau_cmd_msg.data[0] = Tau_cmd_(0);
				tau_cmd_msg.data[1] = Tau_cmd_(1);
				tau_cmd_msg.data[2] = torque;
				tau_cmd_msg.data[3] = Tau_cmd_(2);
				tau_cmd_msg.data[4] = Tau_cmd_(3);
				tau_cmd_msg.data[5] = Tau_cmd_(4);
				tau_cmd_msg.data[6] = Tau_cmd_(5);
				*/
				pub_tau_cmd_.publish(tau_cmd_msg);
				
				F_des_msg.wrench.force.x = F_des_[0];
				F_des_msg.wrench.force.y = F_des_[1];
				F_des_msg.wrench.force.z = F_des_[2];
				F_des_msg.wrench.torque.x = 0;
				F_des_msg.wrench.torque.y = 0;
				F_des_msg.wrench.torque.z = 0;
				pub_F_des_.publish(F_des_msg);
				
				Sv_msg.data = S_v_(0,0);	
				pub_Sv.publish(Sv_msg);
				Sf_msg.data = S_f_(0,0);	
				pub_Sf.publish(Sf_msg);
				
			ros::Time end = ros::Time::now();
			if(count%100==0)
			{	//std::cout << "time"<<end-begin<< "\n";
				std::cout << "d_wall="<<d_wall_<<" Sf="<<S_f_(0,0)<<" Sv="<<S_v_(0,0)<<"\n";
				//std::cout << "Here is the Jkdl_:\n" << Jkdl_.data<< "\n";
				//std::cout << "Here is the J6x6_:\n" << ((J6x6_.transpose()).inverse())*maxtorques<< "\n";
				//std::cout << "Here is the J_trans_:\n" << J_trans_<< "\n";
				//std::cout << "Here is the J_inv_:\n" << J_inv_<< "\n";
				//std::cout << "Here is the M_.data:\n" << M_.data<< "\n";
				//std::cout << "Here is the I6x6_:\n" << I6x6_<< "\n";
				//std::cout << "out:\n" << LAMDA_<< "\n";
				//ROS_INFO("trajectory-> x=%lf y=%lf z=%lf \n",traj_des_.q.data(1),traj_des_.q.data(2),traj_des_.q.data(3));
				//ROS_INFO("Current position x=%lf y=%lf z=%lf ",P_current_.p.x(),P_current_.p.y(),P_current_.p.z());
				 //ROS_INFO("Current position q1=%lf,q2=%lf, q1+q2 = %lf, q3=%lf  ",joint_handles_[1].getPosition(),joint_handles_[3].getPosition(),joint_handles_[1].getPosition()+joint_handles_[3].getPosition(),joint_handles_[5].getPosition() );
				//ROS_INFO("q5=%lf q6=%lf q7=%lf",joint_msr_states_.q(4),joint_msr_states_.q(5),joint_msr_states_.q(6));

				/*ROS_INFO("***** Current velocity **************");
				ROS_INFO("x=%lf y=%lf z=%lf ",V_current_.GetTwist ().vel.x(),V_current_.GetTwist ().vel.y(),V_current_.GetTwist ().vel.z());*/
							
				//ROS_INFO("x=%lf",traj_resp_msg.pos.x);
			}				

		}
        
	}
	
	
	void TorqueBasedPositionControllerGazebo:: calculatePsuedoInertia(const Eigen::MatrixXd  & j, const Eigen::MatrixXd & i, Eigen::MatrixXd & lamda ){
		
		lamda = ((j)*(i.inverse())*(j.transpose())).inverse();
	}
	
	void TorqueBasedPositionControllerGazebo::transformKDLToEigen_(const KDL::Frame  & frame, Eigen::MatrixXd & matrix) const
	{
	
		for (size_t i=0; i<3; ++i)
			matrix(i,0) = frame.p(i);
		
		for (size_t i=3; i<6; ++i)
			matrix(i,0) = 0.0;
		
	}
	void TorqueBasedPositionControllerGazebo:: calculateAccelerationCommand(const KDL::Frame  & p_current, const KDL::FrameVel & v_current, const KDL::JntArrayAcc & traj_des, Eigen::MatrixXd & alpha_v )
	{
		
		MatrixXd p_resp(6,1);
		MatrixXd v_resp(6,1);
		p_resp = MatrixXd::Zero(6, 1);
		v_resp = MatrixXd::Zero(6, 1);
		
		for (size_t i=0; i<3; ++i)
		{
			p_resp(i,0) = p_current.p(i);
			v_resp(i,0) = v_current.p.v(i);
		}
	
		alpha_v = (traj_des_.qdotdot.data + KDv_*(traj_des.qdot.data - v_resp)  + KPv_*(traj_des.q.data - p_resp));
	}
	void TorqueBasedPositionControllerGazebo :: SelectionMatricesTransitionControl()
	{
		double offset    = 0.278;              // offset between radars and wall at distance = 0
		double Dist      = d_wall_ - offset;	
		
		if (Dist < 0) 
		{
			Dist = 0;
		}
		
		double S_f_0     = 1;
		double x_initial = 0;
		double x_final   = 2.5;   // domain of work
		double S_f_final = 0.0001;
		double gain      = 10000;
		double eps       = (x_final - x_initial)*gain; // value to adjust the shape of the transition curve
		double ka        = (-log(S_f_final/S_f_0)/x_final )+eps;
		
		S_f_(0,0) = S_f_0*exp(-ka*Dist); // the term corresponding to the direction controlled by position then force
		S_v_(0,0) = 1-S_f_(0,0);
		//std::cout<<"Sf "<< S_f_0*exp(-ka*Dist)<<"\n";
	}
	
	void TorqueBasedPositionControllerGazebo ::calculateForceCommand(const Eigen::VectorXd & FT_sensor, const Eigen::VectorXd & F_des, Eigen::MatrixXd & F_cmd )
	{
		F_cmd_ = F_des + KPf_*(F_des + FT_sensor);  // pay attention to the sign of the force sensor

	}
	
	void TorqueBasedPositionControllerGazebo :: calculateJointTorques(const Eigen::MatrixXd & j6x6,const Eigen::MatrixXd & alpha_v, Eigen::VectorXd & Tau_cmd_  )
	{
		//Tau_cmd_ = Jkdl_.data.transpose()*(LAMDA_*S_v_*alpha_v);
		Tau_cmd_ = J6x6_.transpose()*((LAMDA_*S_v_*alpha_v)+(S_f_*F_cmd_));
	}
	void TorqueBasedPositionControllerGazebo :: removeColumn(Eigen::MatrixXd& matrix_in, unsigned int colToRemove)
	{
		unsigned int numRows = matrix_in.rows();
		unsigned int numCols = matrix_in.cols()-1;

		if( colToRemove < numCols )
			matrix_in.block(0,colToRemove,numRows,numCols-colToRemove) = matrix_in.block(0,colToRemove+1,numRows,numCols-colToRemove);
			
		matrix_in.conservativeResize(numRows,numCols);
	}
	
	void TorqueBasedPositionControllerGazebo::removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
	{
		unsigned int numRows = matrix.rows()-1;
		unsigned int numCols = matrix.cols();

		if( rowToRemove < numRows )
			matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

		matrix.conservativeResize(numRows,numCols);
	}
	
	void TorqueBasedPositionControllerGazebo::commandCB(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start commandCB of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			cmd_flag_ = 0;
			return; 
		}
		
		q_des_.resize(msg->data.size());
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			q_des_(i) = (double)msg->data[i]*0;
		}
		
		cmd_flag_ = 1;
		
	}
	
	
	void TorqueBasedPositionControllerGazebo::TrajPathPointCB(const kuka_lwr_controllers::TrajPathPoint::ConstPtr & msg)
    {
		/*
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start trajCB of robot %s!",robot_namespace_.c_str());
		#endif
		*/
		traj_des_.resize(6);

			traj_des_.q.data(0) = (double)msg->pos.x;
			traj_des_.q.data(1) = (double)msg->pos.y;
			traj_des_.q.data(2) = (double)msg->pos.z;
					
			traj_des_.qdot.data(0) = (double)msg->vel.x;
			traj_des_.qdot.data(1) = (double)msg->vel.y;
			traj_des_.qdot.data(2) = (double)msg->vel.z;
			
			traj_des_.qdotdot.data(0) = (double)msg->acc.x;
			traj_des_.qdotdot.data(1) = (double)msg->acc.y;
			traj_des_.qdotdot.data(2) = (double)msg->acc.z;
			
		//	ROS_INFO("TorqueBasedPositionController: TrajPathPointCB  msg->pos.x = %f !",(double)msg->pos.x);

		for (std::size_t i=3; i<6; i++)
		{
			traj_des_.q(i) = 0;
			traj_des_.qdot(i) = 0;
			traj_des_.qdotdot(i) = 0;
		}
		cmd_flag_ = 1;
	}
	void TorqueBasedPositionControllerGazebo::ft_readingsCB(const geometry_msgs::WrenchStamped& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start ft_readingsCB of robot %s!",robot_namespace_.c_str());
		#endif
		FT_sensor_[0] = msg.wrench.force.x ;
		FT_sensor_[1] = msg.wrench.force.y ;
		FT_sensor_[2] = msg.wrench.force.z ;
		FT_sensor_[3] = msg.wrench.torque.x ;
		FT_sensor_[4] = msg.wrench.torque.y ;
		FT_sensor_[5] = msg.wrench.torque.z ;

		cmd_flag_ = 1;
	}
	
	void TorqueBasedPositionControllerGazebo:: setjointsKp(const std_msgs::Float64MultiArrayConstPtr& msg)
	{
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setjointsKp of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size() )
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setjointsKp Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		Kp_joints_.resize(joint_handles_.size());
		
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			Kp_joints_(i) = (double)msg->data[i];
		}
	}
	void TorqueBasedPositionControllerGazebo:: setjointsKd(const std_msgs::Float64MultiArrayConstPtr& msg)
	{
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: setjointsKd setjointsKp of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size() )
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setjointsKd Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		Kd_joints_.resize(joint_handles_.size());
		
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			Kd_joints_(i) = (double)msg->data[i];
		}
	}
	void TorqueBasedPositionControllerGazebo::setcartesianKp(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setcartesianKp of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=6)
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setcartesianKp Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		Kp_cartesian_.resize(6);
		
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			Kp_cartesian_(i) = (double)msg->data[i];
			KPv_(i,i) = (double)msg->data[i];
		}
		
	}
	
	void TorqueBasedPositionControllerGazebo::setcartesianKd(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setcartesianKd of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=6)
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setcartesianKd Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		Kd_cartesian_.resize(6);
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			Kd_cartesian_(i) = (double)msg->data[i];
			KDv_(i,i) = (double)msg->data[i];
		}
	}
	
	void TorqueBasedPositionControllerGazebo::setforceKp(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setforceKp of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=6)
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setforceKp Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}

		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			KPf_(i,i) = (double)msg->data[i];
		}
	}	
	void TorqueBasedPositionControllerGazebo::setRollPitchYaw(const  geometry_msgs::PoseStamped& msg)
	{
		d_wall_ = msg.pose.position.z;
		//ROS_INFO("In setRollPitchYaw");
		double sinr = +2.0 * (msg.pose.orientation.w * msg.pose.orientation.x + msg.pose.orientation.y * msg.pose.orientation.z);
		double cosr = +1.0 - 2.0 * (msg.pose.orientation.x * msg.pose.orientation.x + msg.pose.orientation.y * msg.pose.orientation.y);
		roll_ = atan2(sinr, cosr);

		// pitch (y-axis rotation)
		double sinp = +2.0 * (msg.pose.orientation.w * msg.pose.orientation.y - msg.pose.orientation.z * msg.pose.orientation.x);
		if (fabs(sinp) >= 1)
			pitch_ = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		else
		pitch_ = asin(sinp);

		// yaw (z-axis rotation)
		double siny = +2.0 * (msg.pose.orientation.w * msg.pose.orientation.z + msg.pose.orientation.x * msg.pose.orientation.y);
		double cosy = +1.0 - 2.0 * (msg.pose.orientation.y * msg.pose.orientation.y + msg.pose.orientation.z * msg.pose.orientation.z);  
		yaw_ = atan2(siny, cosy);
	}

void TorqueBasedPositionControllerGazebo::setStiffnessDamping(const kuka_lwr_controllers::StiffnessDamping::ConstPtr & msg){
		
		
		if(msg->stiffness.data.size()!=joint_handles_.size()  )
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: stiffness Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->stiffness.data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
			if(msg->damping.data.size()!=joint_handles_.size()  )
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: Damping Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->damping.data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		for (std::size_t i=0; i<msg->stiffness.data.size(); i++)
		{
			stiff_(i) = (double)msg->stiffness.data[i];
			damp_(i) = (double)msg->damping.data[i];
		}
	}
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::TorqueBasedPositionControllerGazebo, controller_interface::ControllerBase)

