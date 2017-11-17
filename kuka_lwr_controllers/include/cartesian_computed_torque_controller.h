#ifndef LWR_CONTROLLERS_CARTESIAN_COMPUTED_TORQUE_CONTROLLER_H
#define LWR_CONTROLLERS_CARTESIAN_COMPUTED_TORQUE_CONTROLLER_H

// Controller base
#include "kinematic_chain_controller_base.h"

// hardware_interface 
#include <kuka_lwr_hw/lwr_kuka_interface.h> // contains definition of KUKAJointInterface

// KDL
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>


#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>

// Ros messages generated
#include <kuka_lwr_controllers/PoseRPY.h>

// FRI Type IRML
//#include <TypeIRML.h>
#include <ReflexxesAPI.h>
#include <RMLPositionFlags.h>
#include <RMLPositionInputParameters.h>
#include <RMLPositionOutputParameters.h>


#ifndef PI
	#define PI 3.1415926535897932384626433832795
#endif

#ifndef RAD
	#define RAD(A)	((A) * PI / 180.0 )
#endif

#ifndef DEG
	#define DEG(A)	((A) * 180.0 / PI )
#endif



namespace kuka_lwr_controllers
{
	class CartesianComputedTorqueController: public controller_interface::KinematicChainControllerBase<hardware_interface::KUKAJointInterface> 
	{
	public:

		CartesianComputedTorqueController();
		~CartesianComputedTorqueController();

		bool init(hardware_interface::KUKAJointInterface *robot, ros::NodeHandle &n);
		void starting(const ros::Time& time);
		void update(const ros::Time& time, const ros::Duration& period);
		//void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void command(const kuka_lwr_controllers::PoseRPY::ConstPtr &msg);
		void set_gains(const std_msgs::Float64MultiArray::ConstPtr &msg);

	private:

		ros::Subscriber sub_posture_;
		ros::Subscriber sub_gains_;
        
		KDL::JntArray cmd_states_;
		int cmd_flag_;	// discriminate if a user command arrived
		double lambda;	// flattening coefficient of tanh
		int step_;		// step used in tanh for reaching gradually the desired posture
		KDL::JntArray joint_initial_states_; // joint as measured at the beginning of the control action
		KDL::JntArray current_cmd_; // command value as delta to be added to joint_initial_states_

		KDL::JntArray tau_cmd_;
		KDL::JntSpaceInertiaMatrix M_; //Inertia matrix
		KDL::JntArray C_, G_;	//Coriolis and Gravitational matrices
		KDL::JntArray Kp_, Kv_;	//Position and Velocity gains

		boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
		boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;  // Jacobian Solver
		boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;  // Forward kinematic Solver (Jnt to Cartesian)
		boost::scoped_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
		boost::scoped_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_;

		KDL::Frame x_;		//current pose
		KDL::Frame x_des_;	//desired pose

		KDL::Twist x_err_;	// error of end effector position and rotation

		KDL::Jacobian J_;	//Jacobian
		Eigen::MatrixXd J_pinv_; // Pseudo Inverse Matrix

		ReflexxesAPI *RML_;
		RMLPositionInputParameters  	*IP_;
		RMLPositionOutputParameters *	OP_;
		RMLPositionFlags            	Flags_;

		
		double cycleTime_;
		int resultValue_;

	};
}

#endif
