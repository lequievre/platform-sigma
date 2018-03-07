/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#ifndef LWR_CONTROLLERS_SIMPLE_CARTESIAN_IMPEDANCE_CONTROLLER_H
#define LWR_CONTROLLERS_SIMPLE_CARTESIAN_IMPEDANCE_CONTROLLER_H

// Controller base
#include "kinematic_chain_controller_base.h"

// hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_cartesian_interface.h> // contains definition of KUKACartesianInterface

namespace kuka_lwr_controllers
{
	class SimpleCartesianImpedanceController: 
	public controller_interface::KinematicChainControllerBase<hardware_interface::KUKACartesianInterface>
	{
		public:
			SimpleCartesianImpedanceController();
			~SimpleCartesianImpedanceController();

			bool init(hardware_interface::KUKACartesianInterface *robot, ros::NodeHandle &n); // Init the controller
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			//void setStiffnessDamping(const kuka_lwr_controllers::StiffnessDamping::ConstPtr & msg);
			//void commandCB(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe command topic
			
		private:
			std::string robot_namespace_;
			hardware_interface::KUKACartesianStiffnessHandle kuka_cart_stiff_handle_;
			hardware_interface::KUKACartesianDampingHandle kuka_cart_damp_handle_;
			
			int trace_count_ = 0;
			
			/*KDL::JntArray  stiff_, damp_, q_des_; // stiffness and damping values got from topic
			ros::Subscriber sub_stiffness_damping_, sub_command_; // subscribers of stiffness and damping and command position
			* */
	};
	
}

#endif
