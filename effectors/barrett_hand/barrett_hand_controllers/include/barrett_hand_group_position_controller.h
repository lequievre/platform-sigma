/*
 *  Laurent LEQUIEVRE
 *  Research Engineer, CNRS (France)
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#ifndef BARRETT_HAND_GROUP_POSITION_CONTROLLER_H
#define BARRETT_HAND_GROUP_POSITION_CONTROLLER_H

#define TRACE_ACTIVATED 0

// ROS
#include <controller_interface/controller.h>

// hardware_interface 
#include <hardware_interface/joint_command_interface.h> // contains definition of PositionJointInterface

// URDF
#include <urdf/model.h>

// KDL
#include <kdl/kdl.hpp>
#include <kdl/jntarrayacc.hpp>

// msgs Float64MultiArray
#include <std_msgs/Float64MultiArray.h>

// ros tools for realtime_buffer
#include <realtime_tools/realtime_buffer.h>

namespace barrett_hand_controllers
{
	class BarrettHandGroupPosition: public controller_interface::Controller<hardware_interface::PositionJointInterface>
	{
		public:
			BarrettHandGroupPosition();
			~BarrettHandGroupPosition();
			
			bool init(hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n); // Init the controller
			
			void starting(const ros::Time& time);  // Start the controller
			void stopping(const ros::Time& time);  // Stop the controller
			void update(const ros::Time& time, const ros::Duration& period);  // Update the controller
			
		private:
			std::string robot_namespace_;
			ros::NodeHandle nh_;
			
			// configuration
			int n_joints_ = 8; // the barret hand has 8 joints maximum
			int n_dof_ = 4; // 4 degrees of freedom
			std::vector<std::string> joint_names_; // vector of joints names
			
			std::vector<hardware_interface::PositionJointInterface::ResourceHandleType> joint_handles_;
			
			struct limits_
			{
				KDL::JntArray min;
				KDL::JntArray max;
			} joint_limits_; // KDL structures to store limits min, max each joint
			
			void commandCB_(const std_msgs::Float64MultiArrayConstPtr& msg); // function associate to a subscribe command topic
			std::vector<std::string> getStrings_(const ros::NodeHandle& nh, const std::string& param_name);
			
			ros::Subscriber sub_command_;
			realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_; // the vector of desired joint values
	}; // End of class BarrettHandGroupPosition
}

#endif
