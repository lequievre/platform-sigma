/*
 *  Laurent LEQUIEVRE
 *  Institut Pascal UMR6602
 *  laurent.lequievre@univ-bpclermont.fr
 * 
*/

#include <simple_cartesian_impedance_controller.h>

// For plugin
#include <pluginlib/class_list_macros.h>

namespace kuka_lwr_controllers 
{
    SimpleCartesianImpedanceController::SimpleCartesianImpedanceController() {}
    SimpleCartesianImpedanceController::~SimpleCartesianImpedanceController() {}
    
     bool SimpleCartesianImpedanceController::init(hardware_interface::KUKACartesianInterface *robot, ros::NodeHandle &n)
     {
		robot_namespace_ = n.getNamespace();
		// robot_namespace_ contains the namespace concatenate with the name of this controller
		// example : /kuka_lwr_left/kuka_simple_cartesian_impedance_controller
		// Only need to get the namespace --------------------------------------------
		int pos_found = robot_namespace_.find("/",1);
		std::string robot_namespace_only = robot_namespace_.substr(1,pos_found-1);
		// ----------------------------------------------------------------------------
		 
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Start init of robot %s !",robot_namespace_.c_str());
			ROS_INFO("SimpleCartesianImpedanceController: Start init of robot ns only %s !",robot_namespace_only.c_str());
		#endif
		
			
        if( !(KinematicChainControllerBase<hardware_interface::KUKACartesianInterface>::init(robot, n)) )
        {
            ROS_ERROR("SimpleCartesianImpedanceController: Couldn't initilize SimpleCartesianImpedanceController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        
        // Get Cartesian Stiffness and Damping Interfaces Handles
        kuka_cart_stiff_handle_ = robot->getHandle(robot_namespace_only + "_cart_stiffness");
        kuka_cart_damp_handle_ = robot->getHandle(robot_namespace_only + "_cart_damping");
        
        sub_cart_stiffness_command_ = n.subscribe("setCartesianStiffness", 1, &SimpleCartesianImpedanceController::setCartesianStiffness, this); 
        sub_cart_damping_command_ = n.subscribe("setCartesianDamping", 1, &SimpleCartesianImpedanceController::setCartesianDamping, this); 
       
        return true;
        
	 }
	 
	 void SimpleCartesianImpedanceController::starting(const ros::Time& time)
     {
        
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		
		trace_count_ = 0;
     }
    
     void SimpleCartesianImpedanceController::stopping(const ros::Time& time)
	 {
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO_NAMED("SimpleCartesianImpedanceController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
	 }
	 
	 void SimpleCartesianImpedanceController::update(const ros::Time& time, const ros::Duration& period)
     {
		 #if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			 trace_count_++;
			 if (trace_count_%10000)
			 {
				ROS_INFO("-> stiffness -> X = %f, Y = %f, Z = %f, A = %f, B = %f, C = %f", kuka_cart_stiff_handle_.getX(), kuka_cart_stiff_handle_.getY(), kuka_cart_stiff_handle_.getZ(), kuka_cart_stiff_handle_.getA(), kuka_cart_stiff_handle_.getB(), kuka_cart_stiff_handle_.getC());
				/*for (int i=0; i<7; i++)
				{
					ROS_INFO("Joint Pos(%d) = %f", i, joint_handles_[i].getPosition());
				}*/
			 }
		 #endif
	 }
	 
	 void SimpleCartesianImpedanceController::setCartesianStiffness(const std_msgs::Float64MultiArrayConstPtr& msg)
	 {
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Start setCartesianStiffness of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=NUMBER_OF_CART_DOFS)
		{ 
			ROS_ERROR_STREAM("SimpleCartesianImpedanceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of DOF(" << NUMBER_OF_CART_DOFS << ")! Not executing!");
			return; 
		}
		
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("Get SimpleCartesianImpedanceController/setCartesianStiffness X=%f, Y=%f, Z=%f, A=%f, B=%f, C=%f",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
		#endif
		
		// 0: X, 1: Y, 2: Z, 3: A, 4: B, 5: C
		kuka_cart_stiff_handle_.setCommandX(msg->data[0]);
		kuka_cart_stiff_handle_.setCommandY(msg->data[1]);
		kuka_cart_stiff_handle_.setCommandZ(msg->data[2]);
		kuka_cart_stiff_handle_.setCommandA(msg->data[3]);
		kuka_cart_stiff_handle_.setCommandB(msg->data[4]);
		kuka_cart_stiff_handle_.setCommandC(msg->data[5]);
		
	 }
	 
	 void SimpleCartesianImpedanceController::setCartesianDamping(const std_msgs::Float64MultiArrayConstPtr& msg)
	 {
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Start setCartesianDamping of robot %s!",robot_namespace_.c_str());
		#endif

		if(msg->data.size()!=NUMBER_OF_CART_DOFS)
		{ 
			ROS_ERROR_STREAM("SimpleCartesianImpedanceController: Dimension (of robot " << robot_namespace_.c_str() << ") of command (" << msg->data.size() << ") does not match number of DOF(" << NUMBER_OF_CART_DOFS << ")! Not executing!");
			return; 
		}
		
		#if TRACE_CARTESIAN_IMPENDANCE_CONTROLLER_ACTIVATED
			ROS_INFO("Get SimpleCartesianImpedanceController/setCartesianDamping X=%f, Y=%f, Z=%f, A=%f, B=%f, C=%f",msg->data[0],msg->data[1],msg->data[2],msg->data[3],msg->data[4],msg->data[5]);
		#endif
		
		// 0: X, 1: Y, 2: Z, 3: A, 4: B, 5: C
		kuka_cart_damp_handle_.setCommandX(msg->data[0]);
		kuka_cart_damp_handle_.setCommandY(msg->data[1]);
		kuka_cart_damp_handle_.setCommandZ(msg->data[2]);
		kuka_cart_damp_handle_.setCommandA(msg->data[3]);
		kuka_cart_damp_handle_.setCommandB(msg->data[4]);
		kuka_cart_damp_handle_.setCommandC(msg->data[5]);
		
	 }
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::SimpleCartesianImpedanceController, controller_interface::ControllerBase)
