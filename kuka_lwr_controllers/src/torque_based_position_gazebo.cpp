/*
 *  Kamal mohy el dine 
 *  Institut Pascal UMR6602
 *  kamal.mohy.el.dine@gmail.com
 * 
*/

#include <torque_based_position_gazebo.h>

// For plugin
#include <pluginlib/class_list_macros.h>

#include <algorithm>


namespace kuka_lwr_controllers 
{
    TorqueBasedPositionControllerGazebo::TorqueBasedPositionControllerGazebo() {}
    TorqueBasedPositionControllerGazebo::~TorqueBasedPositionControllerGazebo() {}

    bool TorqueBasedPositionControllerGazebo::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
    {
		
		robot_namespace_ = n.getNamespace();
		
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start init of robot %s !",robot_namespace_.c_str());
		#endif
			
        if( !(KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n)) )
        {
            ROS_ERROR("TorqueBasedPositionController: Couldn't initilize TorqueBasedPositionController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        
        sub_command_ = n.subscribe("command", 1, &TorqueBasedPositionControllerGazebo::commandCB, this);
        sub_kp_ = n.subscribe("setKp", 1, &TorqueBasedPositionControllerGazebo::setKp, this);
        sub_kd_ = n.subscribe("setKd", 1, &TorqueBasedPositionControllerGazebo::setKd, this);

		cmd_flag_ = 0;  // set this flag to 0 to not run the update method
		
		Kp_.resize(joint_handles_.size());
		Kd_.resize(joint_handles_.size());
		
		for (std::size_t i=0; i<joint_handles_.size()-3; i++)
		{
			Kp_(i) = 100.0;
			Kd_(i) = 50.0;
		}
		
		for (std::size_t i=4; i<joint_handles_.size(); i++)
		{
			Kp_(i) = 50.0;
			Kd_(i) = 10.0;
		}
		
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
		
		double torque;
		
		if (cmd_flag_)
		{
		
			// update the commanded position to the actual, so that the robot doesn't 
			// go back at full speed to the last commanded position when the stiffness 
			// is raised again
			for(size_t i=0; i<joint_handles_.size(); i++) 
			{
				//ROS_INFO("joint 0 Position -> %f",joint_handles_[0].getPosition());
				//ROS_INFO("joint 0 Velocity -> %f",joint_handles_[0].getVelocity());
				torque = (Kp_(i)*(q_des_(i)-joint_handles_[i].getPosition()))+(Kd_(i)*(-joint_handles_[i].getVelocity()));
				
				joint_handles_[i].setCommand(torque); // Set a value of torque to 0.0 for each joint.
				
			}
		}
        
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
			q_des_(i) = (double)msg->data[i];
		}
		
		cmd_flag_ = 1;
		
	}
	
	void TorqueBasedPositionControllerGazebo::setKp(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setKp of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setKp Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		Kp_.resize(joint_handles_.size());
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			Kp_(i) = (double)msg->data[i];
		}
		
	}
	
	void TorqueBasedPositionControllerGazebo::setKd(const std_msgs::Float64MultiArrayConstPtr& msg)
    {
		#if TRACE_Torque_Based_Position_ACTIVATED
			ROS_INFO("TorqueBasedPositionController: Start setKd of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=joint_handles_.size())
		{ 
			ROS_ERROR_STREAM("TorqueBasedPositionController: setKd Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of joints (" << joint_handles_.size() << ")! Not executing!");
			return; 
		}
		
		Kd_.resize(joint_handles_.size());
		for (std::size_t i=0; i<msg->data.size(); i++)
		{
			Kd_(i) = (double)msg->data[i];
		}
		
	}
	
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::TorqueBasedPositionControllerGazebo, controller_interface::ControllerBase)

