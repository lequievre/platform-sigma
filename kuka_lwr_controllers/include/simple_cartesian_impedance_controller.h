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

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>

// KDL added
#include <kdl/stiffness.hpp>
#include <kdl/trajectory.hpp>
#include <kdl_conversions/kdl_msg.h>

#define TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED 0

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
			void setCartesianStiffness(const std_msgs::Float64MultiArrayConstPtr& msg);
			void setCartesianDamping(const std_msgs::Float64MultiArrayConstPtr& msg);
			
			
		private:
			std::string robot_namespace_;
			hardware_interface::KUKACartesianStiffnessHandle kuka_cart_stiff_handle_;
			hardware_interface::KUKACartesianDampingHandle kuka_cart_damp_handle_;
			hardware_interface::KUKACartesianPoseHandle kuka_cart_pose_handle_;
			hardware_interface::KUKACartesianWrenchHandle kuka_cart_wrench_handle_;
			
			int trace_count_ = 0;
			
			ros::Subscriber sub_cart_stiffness_command_, sub_cart_damping_command_;
			
			// Utility function to get the current pose
			void getCurrentPose_(KDL::Frame& frame);
			
			// FRI<->KDL conversion
			void fromKDLtoFRI_(const KDL::Frame& frame_in, std::vector<double>& vect_Pose_FRI_out);
			
			// Utility function to forward commands to FRI
			void forwardCmdFRI_(const KDL::Frame& frame, const KDL::Stiffness& stiffness, const KDL::Stiffness& damping, const KDL::Wrench& wrench);
			
			KDL::Frame pose_cur_;
			KDL::Stiffness stiff_cur_, damp_cur_;
			KDL::Wrench wrench_cur_;
			std::vector<double> cur_Pose_FRI_;
			
			static constexpr int NUMBER_OF_CART_DOFS = 6;
			static constexpr int NUMBER_OF_FRAME_ELEMENTS = 12;
			
	};
	
}

#endif
