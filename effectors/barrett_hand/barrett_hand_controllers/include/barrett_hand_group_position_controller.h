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

// srvs SetBool
#include <std_srvs/SetBool.h>

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
		
			// Useful to convert enum Class to int (necessary if you need to use enum Class as index of array)
			template <typename T>
			constexpr typename std::underlying_type<T>::type enum_value(T val)
			{
				return static_cast<typename std::underlying_type<T>::type>(val);
			}
			
			
			// Define "Break Away" option for grasping.
			enum class GraspingMode : int
			{
				Break_Away_Active = 0,
				Break_Away_Inactive = 1
			};
			
			// Define index value of each joints.
			enum class Fingers : int
			{
				Finger1_KNUCKLE = 0,
				Finger1_PROXIMAL = 1,
				Finger1_DISTAL = 2,
				Finger2_KNUCKLE = 3,
				Finger2_PROXIMAL = 4,
				Finger2_DISTAL = 5,
				Finger3_PROXIMAL = 6,
				Finger3_DISTAL = 7
			};
		
			std::string robot_namespace_;
			ros::NodeHandle nh_; // Node Handle of controller
			
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
			bool setBreakAwayCB_(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
			void checkLimit_(double &value, const Fingers& index) const noexcept;
			std::vector<std::string> getStrings_(const ros::NodeHandle& nh, const std::string& param_name) const noexcept;
			
			ros::Subscriber sub_command_;
			ros::ServiceServer srv_command_;
			realtime_tools::RealtimeBuffer<std::vector<double> > commands_buffer_; // the vector of desired joint values
			realtime_tools::RealtimeBuffer<GraspingMode> grasping_mode_buffer_;
	}; // End of class BarrettHandGroupPosition
}

#endif
