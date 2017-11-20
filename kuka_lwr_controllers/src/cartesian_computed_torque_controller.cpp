#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <cartesian_computed_torque_controller.h>

// Utils for pseudo inverse and skew_symmetric
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <math.h>       /* sqrt */

namespace kuka_lwr_controllers
{
	CartesianComputedTorqueController::CartesianComputedTorqueController() {}
	CartesianComputedTorqueController::~CartesianComputedTorqueController() {
			delete RML_;
			delete IP_;
			delete OP_;	
	}

	bool CartesianComputedTorqueController::init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n)
	{
        KinematicChainControllerBase<hardware_interface::KUKAJointInterface>::init(robot, n);
        
		id_solver_.reset( new KDL::ChainDynParam( kdl_chain_, gravity_) );

		cmd_states_.resize(kdl_chain_.getNrOfJoints());
		tau_cmd_.resize(kdl_chain_.getNrOfJoints());
		Kp_.resize(kdl_chain_.getNrOfJoints());
		Kv_.resize(kdl_chain_.getNrOfJoints());
		M_.resize(kdl_chain_.getNrOfJoints());
		C_.resize(kdl_chain_.getNrOfJoints());
		G_.resize(kdl_chain_.getNrOfJoints());
        current_cmd_.resize(kdl_chain_.getNrOfJoints());

		sub_posture_ = nh_.subscribe("command", 1, &CartesianComputedTorqueController::command, this);
		sub_gains_ = nh_.subscribe("set_gains", 1, &CartesianComputedTorqueController::set_gains, this);

		fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
		ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
		ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_,joint_limits_.min,joint_limits_.max,*fk_pos_solver_,*ik_vel_solver_));

		cycleTime_ = 0.002;
        
		RML_ = new ReflexxesAPI(joint_handles_.size(),cycleTime_);
		IP_ = new RMLPositionInputParameters(joint_handles_.size());
		OP_ = new RMLPositionOutputParameters(joint_handles_.size());

		return true;		
	}

	void CartesianComputedTorqueController::starting(const ros::Time& time)
	{
  		// get joint positions
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{

  			Kp_(i) = 1000.0;
  			Kv_(i) = 300.0;
    			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    			joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration();
    			joint_des_states_.q(i) = joint_msr_states_.q(i);
    		}

    		lambda = 0.1;	// lower values: flatter
    		cmd_flag_ = 0;

    		ROS_INFO(" Number of joints in handle = %lu", joint_handles_.size() );

    }

    void CartesianComputedTorqueController::update(const ros::Time& time, const ros::Duration& period)
    {
    	// get joint positions
		for(size_t i=0; i<joint_handles_.size(); i++) 
		{
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
			joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration();
		}

		//ROS_INFO("-> msr values : %f, %f, %f, %f, %f, %f, %f!",joint_msr_states_.q(0), joint_msr_states_.q(1), joint_msr_states_.q(2), joint_msr_states_.q(3), joint_msr_states_.q(4), joint_msr_states_.q(5), joint_msr_states_.q(6));

    	if(cmd_flag_)
    	{
				// Calculates the joint values 'cmd_states_' that correspond to the input pose 'x_des_' 
				// given an initial guess 'joint_msr_states_.q'. 
				ik_pos_solver_->CartToJnt(joint_msr_states_.q, x_des_, cmd_states_);

				// Use reflexxes type 2 library that help to generate a 'smooth' movement. 
				for (size_t i=0; i<joint_handles_.size(); ++i)
				{
					IP_->CurrentPositionVector->VecData[i] = joint_msr_states_.q(i);  // set current position with current position of joint handles
					IP_->TargetPositionVector->VecData[i]	= cmd_states_(i); // set desired position (get from inverse kin solver)
					IP_->MaxVelocityVector->VecData[i] = (double)1.0;
					IP_->MaxAccelerationVector->VecData[i] = (double)4.0;
					IP_->MaxJerkVector->VecData[i] = (double)8.0;
					IP_->SelectionVector->VecData[i] = true;
				}

				// Get next 'smooth' position in 'OP_' object. 
				resultValue_  =   RML_->RMLPosition(*IP_,OP_,Flags_);
				
				if (resultValue_ < 0)
				{
					ROS_INFO("GroupCommandControllerFRI::update : ERROR during trajectory generation err n°%d",resultValue_);
				}
				
				// set next desired 'smooth' states : position, velocity, acceleration
				for (int i = 0; i < joint_handles_.size(); i++)
				{
					joint_des_states_.q(i) = OP_->NewPositionVector->VecData[i];
					joint_des_states_.qdot(i) = OP_->NewVelocityVector->VecData[i];
					joint_des_states_.qdotdot(i) = OP_->NewAccelerationVector->VecData[i];

				}

				// computing forward kinematics. Transform 'q' angle position to 3D pose in 'x_'.
				// It is necessary to get the 'euler' distance from 'x_des_' to 'x_'.
				fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

				*IP_->CurrentPositionVector      =   *OP_->NewPositionVector      ;
				*IP_->CurrentVelocityVector      =   *OP_->NewVelocityVector      ;
				*IP_->CurrentAccelerationVector  =   *OP_->NewAccelerationVector  ;

				// Is the distance from 'x_des_' to 'x_' is close to 0.005 meter ?
				if (Equal(x_, x_des_, 0.005))
				{
						ROS_INFO("On target");
						// Computing 'euler' distance between measure (x_) to desired (x_des_).
						double distance = sqrt(pow(x_des_.p.x()-x_.p.x(),2) + pow(x_des_.p.y()-x_.p.y(),2) + pow(x_des_.p.z()-x_.p.z(),2));
						ROS_INFO("distance = %f\n",distance);  // Show the distance only for information.
						cmd_flag_ = 0;
				}

    	}

    	// computing Inertia, Coriolis and Gravity matrices
    	id_solver_->JntToMass(joint_msr_states_.q, M_);
    	id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
    	id_solver_->JntToGravity(joint_msr_states_.q, G_);

        // PID controller
        KDL::JntArray pid_cmd_(joint_handles_.size());
        
        // Compensation of Coriolis and Gravity
        KDL::JntArray cg_cmd_(joint_handles_.size());
        
        for(size_t i=0; i<joint_handles_.size(); i++)
        {
            // PID control law.
            pid_cmd_(i) = joint_des_states_.qdotdot(i) + Kv_(i)*(joint_des_states_.qdot(i) - joint_msr_states_.qdot(i)) + Kp_(i)*(joint_des_states_.q(i) - joint_msr_states_.q(i));

			//pid_cmd_(i) =  Kv_(i)*(joint_des_states_.qdot(i) - joint_msr_states_.qdot(i)) + Kp_(i)*(joint_des_states_.q(i) - joint_msr_states_.q(i));
            //cg_cmd_(i) = C_(i)*joint_msr_states_.qdot(i) + G_(i);
            
            // Global control law
            cg_cmd_(i) = C_(i) + G_(i);
        }
        
        // Torque command : 'tau_cmd_'.
        tau_cmd_.data = M_.data * pid_cmd_.data;
        KDL::Add(tau_cmd_,cg_cmd_,tau_cmd_);
        
		//ROS_INFO("-> tau cmd values : %f, %f, %f, %f, %f, %f, %f!",tau_cmd_(0), tau_cmd_(1), tau_cmd_(2), tau_cmd_(3), tau_cmd_(4),tau_cmd_(5), tau_cmd_(6));

        for(size_t i=0; i<joint_handles_.size(); i++)
        {
            joint_handles_[i].setCommandTorque(tau_cmd_(i));
			joint_handles_[i].setCommandPosition(joint_msr_states_.q(i));
			//joint_handles_[i].setCommandPosition(pid_cmd_(i));
			//joint_handles_[i].setCommandPosition(joint_des_states_.q(i));
        }
    }

    void CartesianComputedTorqueController::command(const kuka_lwr_controllers::PoseRPY::ConstPtr &msg)
    {
		ROS_INFO("***** START OneTaskInverseKinematics::command ************");

        KDL::Frame frame_des;

        switch(msg->id)
        {
            case 0:
				ROS_INFO("***** CartesianComputedTorqueController::command position and orientation ************");
				//ROS_INFO("position desired -> x = %f, y = %f, z = %f", msg->position.x, msg->position.y, msg->position.z);
				frame_des = KDL::Frame(
                    KDL::Rotation::RPY(msg->orientation.roll,
                                      msg->orientation.pitch,
                                      msg->orientation.yaw),
                    KDL::Vector(msg->position.x,
                                msg->position.y,
                                msg->position.z));
            break;

            case 1: // position only
				ROS_INFO("***** CartesianComputedTorqueController::command position only ************");
				frame_des = KDL::Frame(
                KDL::Vector(msg->position.x,
                            msg->position.y,
                            msg->position.z));
            break;

            case 2: // orientation only
				ROS_INFO("***** CartesianComputedTorqueController::command orientation only ************");
				frame_des = KDL::Frame(
                KDL::Rotation::RPY(msg->orientation.roll,
                                   msg->orientation.pitch,
                                   msg->orientation.yaw));
            break;

            default:
				ROS_INFO("Wrong message ID");
            return;
        }

        x_des_ = frame_des;
		resultValue_ = ReflexxesAPI::RML_WORKING;
        cmd_flag_ = 1;
        
        ROS_INFO("***** FINISH CartesianComputedTorqueController::command ************");
    }

	void CartesianComputedTorqueController::set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg)
	{
		if(msg->data.size() == 2*joint_handles_.size())
		{
			for(unsigned int i = 0; i < joint_handles_.size(); i++)
			{
				Kp_(i) = msg->data[i];
				Kv_(i) = msg->data[i + joint_handles_.size()];
			}
		}
		else
			ROS_INFO("Number of Joint handles = %lu", joint_handles_.size());

		ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

		ROS_INFO("New gains Kp: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kp_(0), Kp_(1), Kp_(2), Kp_(3), Kp_(4), Kp_(5), Kp_(6));
		ROS_INFO("New gains Kv: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf, %.1lf", Kv_(0), Kv_(1), Kv_(2), Kv_(3), Kv_(4), Kv_(5), Kv_(6));

	}
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::CartesianComputedTorqueController, controller_interface::ControllerBase)
