#ifndef LWR_CONTROLLERS_BASIC_TORQUE_CONTROLLER_H
#define LWR_CONTROLLERS_BASIC_TORQUE_CONTROLLER_H

// Controller base
#include "kinematic_chain_controller_base.h"

// hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_interface.h> // contains definition of KUKAJointInterface

// Boost
#include <boost/scoped_ptr.hpp>

// Activate Trace info
#define TRACE_Basic_Torque_Controller_ACTIVATED 1

namespace kuka_lwr_controllers
{
	class BasicTorqueController: public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface> 
	{
		public:

			BasicTorqueController();
			~BasicTorqueController();

			bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);	// Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period); // Update the controller
			
			//void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
			//void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);

		private:
		
			std::string robot_namespace_;	// the current robot namespace
			
			int cmd_flag_; // flag set only to 1 when the controller receive a message to the command topic	
	};
}


#endif
