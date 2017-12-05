#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <math.h>

#include <basic_torque_controller.h>

namespace kuka_lwr_controllers
{
	
	BasicTorqueController::BasicTorqueController() {}
	BasicTorqueController::~BasicTorqueController() {}
	
	bool BasicTorqueController::init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n)
	{
		robot_namespace_ = n.getNamespace();
		
		if( !(KinematicChainControllerBase<hardware_interface::KUKAJointInterface>::init(robot, n)) )
		{
            ROS_ERROR("BasicTorqueController: Couldn't initilize BasicTorqueController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
		 
		return true;
	}
	
	
	void BasicTorqueController::stopping(const ros::Time& time)
	{
		#if TRACE_Basic_Torque_Controller_ACTIVATED
			ROS_INFO("BasicTorqueController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
	}
	
	
	void BasicTorqueController::starting(const ros::Time& time)
	{
		#if TRACE_Basic_Torque_Controller_ACTIVATED
			ROS_INFO("BasicTorqueController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		
		// get joint positions
		// KDL::JntArrayAcc -> joint_msr_states_, joint_des_states_
  		for(size_t i=0; i<joint_handles_.size(); i++) 
  		{
    		joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    		joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    		joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration();
    		joint_des_states_.q(i) = joint_msr_states_.q(i);
    	}
    	
    	cmd_flag_ = 0;
    	
		#if TRACE_Basic_Torque_Controller_ACTIVATED
			ROS_INFO(" Number of joints in handle = %lu", joint_handles_.size() );
		#endif
	}
	
	
	void BasicTorqueController::update(const ros::Time& time, const ros::Duration& period)
	{
    	// get joint positions
    	// KDL::JntArrayAcc -> joint_msr_states_
		for(size_t i=0; i<joint_handles_.size(); i++) 
		{
			joint_msr_states_.q(i) = joint_handles_[i].getPosition();
			joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
			joint_msr_states_.qdotdot(i) = joint_handles_[i].getAcceleration();
		}
	}
	
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::BasicTorqueController, controller_interface::ControllerBase)
