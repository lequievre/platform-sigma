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
		
		int pos_found = robot_namespace_.find("/",1);
		std::string robot_namespace_only = robot_namespace_.substr(1,pos_found-1);
		 
		//#if TRACE_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Start init of robot %s !",robot_namespace_.c_str());
			ROS_INFO("SimpleCartesianImpedanceController: Start init of robot ns only %s !",robot_namespace_only.c_str());
		//#endif
		
			
        if( !(KinematicChainControllerBase<hardware_interface::KUKACartesianInterface>::init(robot, n)) )
        {
            ROS_ERROR("SimpleCartesianImpedanceController: Couldn't initilize SimpleCartesianImpedanceController controller of robot %s!",robot_namespace_.c_str());
            return false;
        }
        
        kuka_cart_stiff_handle_ = robot->getHandle(robot_namespace_only + "_cart_stiffness");
        kuka_cart_damp_handle_ = robot->getHandle(robot_namespace_only + "_cart_damping");
       
        return true;
        
	 }
	 
	 void SimpleCartesianImpedanceController::starting(const ros::Time& time)
     {
        
		#if TRACE_ACTIVATED
			ROS_INFO("SimpleCartesianImpedanceController: Starting of robot %s !",robot_namespace_.c_str());
		#endif
		
		trace_count_ = 0;
     }
    
     void SimpleCartesianImpedanceController::stopping(const ros::Time& time)
	 {
		#if TRACE_ACTIVATED
			ROS_INFO_NAMED("SimpleCartesianImpedanceController: Stopping of robot %s !",robot_namespace_.c_str());
		#endif
		
	 }
	 
	 void SimpleCartesianImpedanceController::update(const ros::Time& time, const ros::Duration& period)
     {
		 trace_count_++;
		 if (trace_count_%1000)
		 {
			ROS_INFO("X stiff = %f, A stiff = %f", kuka_cart_stiff_handle_.getX(), kuka_cart_stiff_handle_.getA());
			for (int i=0; i<7; i++)
			{
				ROS_INFO("Joint Pos(%d) = %f", i, joint_handles_[i].getPosition());
			}
		 }
	 }
}

PLUGINLIB_EXPORT_CLASS(kuka_lwr_controllers::SimpleCartesianImpedanceController, controller_interface::ControllerBase)
