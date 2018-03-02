/*
 * Laurent LEQUIEVRE
 * Ingénieur CNRS
 * laurent.lequievre@uca.fr
 * Institut Pascal UMR6602
 * 
*/

#ifndef HARDWARE_INTERFACE_LWR_KUKA_CARTESIAN_INTERFACE_H
#define HARDWARE_INTERFACE_LWR_KUKA_CARTESIAN_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface
{
	/** A KUKA cartesian stiffness state handle class. */
	class KukaCartesianStiffnessStateHandle
	{
		public:
		  KukaCartesianStiffnessStateHandle(): name_(), x_(0), y_(0), z_(0), a_(0), b_(0), c_(0) {}

		  /**
		   * \param name The name of the stiffness handle
		   */
		  KukaCartesianStiffnessStateHandle(const std::string& name, const double* x, const double* y, const double* z, const double* a, const double* b, const double* c)
			: name_(name), x_(x), y_(y), z_(z), a_(a), b_(b), c_(c)
		  {
				if (!x)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. X data pointer is null.");
				}
				if (!y)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. Y data pointer is null.");
				}
				if (!z)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. Z data pointer is null.");
				}
				if (!a)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. A data pointer is null.");
				}
				if (!b)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. B data pointer is null.");
				}
				if (!c)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Siffness State handle '" + name + "'. C data pointer is null.");
				}
		  }

		  std::string getName()     const {return name_;}
		  double getX()      		const {assert(x_); return *x_;}
		  double getY()      		const {assert(y_); return *y_;}
		  double getZ()      		const {assert(z_); return *z_;}
		  double getA()      		const {assert(a_); return *a_;}
		  double getB()      		const {assert(b_); return *b_;}
		  double getC()      		const {assert(c_); return *c_;}

		private:
		  std::string name_;
		  const double* x_;
		  const double* y_;
		  const double* z_;
		  const double* a_;
		  const double* b_;
		  const double* c_;
	};
	
	
	/** A KUKA cartesian damping state handle class. */
	class KukaCartesianDampingStateHandle
	{
		public:
		  KukaCartesianDampingStateHandle(): name_(), x_(0), y_(0), z_(0), a_(0), b_(0), c_(0) {}

		  /**
		   * \param name The name of the stiffness handle
		   */
		  KukaCartesianDampingStateHandle(const std::string& name, const double* x, const double* y, const double* z, const double* a, const double* b, const double* c)
			: name_(name), x_(x), y_(y), z_(z), a_(a), b_(b), c_(c)
		  {
				if (!x)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. X data pointer is null.");
				}
				if (!y)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. Y data pointer is null.");
				}
				if (!z)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. Z data pointer is null.");
				}
				if (!a)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. A data pointer is null.");
				}
				if (!b)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. B data pointer is null.");
				}
				if (!c)
				{
				  throw HardwareInterfaceException("Cannot create Kuka Cartesian Damping State handle '" + name + "'. C data pointer is null.");
				}
		  }

		  std::string getName()     const {return name_;}
		  double getX()      		const {assert(x_); return *x_;}
		  double getY()      		const {assert(y_); return *y_;}
		  double getZ()      		const {assert(z_); return *z_;}
		  double getA()      		const {assert(a_); return *a_;}
		  double getB()      		const {assert(b_); return *b_;}
		  double getC()      		const {assert(c_); return *c_;}

		private:
		  std::string name_;
		  const double* x_;
		  const double* y_;
		  const double* z_;
		  const double* a_;
		  const double* b_;
		  const double* c_;
	};
	
	
	/** \brief A handle used to read and command a KUKA cartesian stiffness. */
	class KUKACartesianStiffnessHandle : public KukaCartesianStiffnessStateHandle
	{
	public:
	  KUKACartesianStiffnessHandle() : KukaCartesianStiffnessStateHandle(), x_cmd_(0), y_cmd_(0), z_cmd_(0), a_cmd_(0), b_cmd_(0), c_cmd_(0) {}

	  /**
	   * \param js This joint's state handle
	   * \param cmd A pointer to the storage for this joint's output command
	   */
	  KUKACartesianStiffnessHandle(const KukaCartesianStiffnessStateHandle& js, double* x_cmd, double* y_cmd, double* z_cmd, double* a_cmd, double* b_cmd, double* c_cmd)
		: KukaCartesianStiffnessStateHandle(js), x_cmd_(x_cmd), y_cmd_(y_cmd), z_cmd_(z_cmd), a_cmd_(a_cmd), b_cmd_(b_cmd), c_cmd_(c_cmd)
	  {
		if (!x_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] X pointer is null.");
		}

		if (!y_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] Y pointer is null.");
		}

		if (!z_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] Z pointer is null.");
		}

		if (!a_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] A pointer is null.");
		}
		
		if (!b_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] B pointer is null.");
		}
		
		if (!c_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Stiffness] C pointer is null.");
		}
	  }

	  void setCommandX(double command)    {assert(x_cmd_); *x_cmd_ = command;}
	  void setCommandY(double command)    {assert(y_cmd_); *y_cmd_ = command;}
	  void setCommandZ(double command)    {assert(z_cmd_); *z_cmd_ = command;}
	  void setCommandA(double command)    {assert(a_cmd_); *a_cmd_ = command;}
	  void setCommandB(double command)    {assert(b_cmd_); *b_cmd_ = command;}
	  void setCommandC(double command)    {assert(c_cmd_); *c_cmd_ = command;}


	  double getCommandX()    const {assert(x_cmd_); return *x_cmd_;}
	  double getCommandY()    const {assert(y_cmd_); return *y_cmd_;}
	  double getCommandZ()    const {assert(z_cmd_); return *z_cmd_;}
	  double getCommandA()    const {assert(a_cmd_); return *a_cmd_;}
	  double getCommandB()    const {assert(b_cmd_); return *b_cmd_;}
	  double getCommandC()    const {assert(c_cmd_); return *c_cmd_;}

	private:

	  double* x_cmd_;
	  double* y_cmd_;
	  double* z_cmd_;
	  double* a_cmd_;
	  double* b_cmd_;
	  double* c_cmd_;

	};
	
	/** \brief A handle used to read and command a KUKA cartesian damping. */
	class KUKACartesianDampingHandle : public KukaCartesianDampingStateHandle
	{
	public:
	  KUKACartesianDampingHandle() : KukaCartesianDampingStateHandle(), x_cmd_(0), y_cmd_(0), z_cmd_(0), a_cmd_(0), b_cmd_(0), c_cmd_(0) {}

	  /**
	   * \param js This joint's state handle
	   * \param cmd A pointer to the storage for this joint's output command
	   */
	  KUKACartesianDampingHandle(const KukaCartesianDampingStateHandle& js, double* x_cmd, double* y_cmd, double* z_cmd, double* a_cmd, double* b_cmd, double* c_cmd)
		: KukaCartesianDampingStateHandle(js), x_cmd_(x_cmd), y_cmd_(y_cmd), z_cmd_(z_cmd), a_cmd_(a_cmd), b_cmd_(b_cmd), c_cmd_(c_cmd)
	  {
		if (!x_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] X pointer is null.");
		}

		if (!y_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] Y pointer is null.");
		}

		if (!z_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] Z pointer is null.");
		}

		if (!a_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] A pointer is null.");
		}
		
		if (!b_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] B pointer is null.");
		}
		
		if (!c_cmd_)
		{
		  throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command [KUKA Cartesian Damping] C pointer is null.");
		}
	  }

	  void setCommandX(double command)    {assert(x_cmd_); *x_cmd_ = command;}
	  void setCommandY(double command)    {assert(y_cmd_); *y_cmd_ = command;}
	  void setCommandZ(double command)    {assert(z_cmd_); *z_cmd_ = command;}
	  void setCommandA(double command)    {assert(a_cmd_); *a_cmd_ = command;}
	  void setCommandB(double command)    {assert(b_cmd_); *b_cmd_ = command;}
	  void setCommandC(double command)    {assert(c_cmd_); *c_cmd_ = command;}


	  double getCommandX()    const {assert(x_cmd_); return *x_cmd_;}
	  double getCommandY()    const {assert(y_cmd_); return *y_cmd_;}
	  double getCommandZ()    const {assert(z_cmd_); return *z_cmd_;}
	  double getCommandA()    const {assert(a_cmd_); return *a_cmd_;}
	  double getCommandB()    const {assert(b_cmd_); return *b_cmd_;}
	  double getCommandC()    const {assert(c_cmd_); return *c_cmd_;}

	private:

	  double* x_cmd_;
	  double* y_cmd_;
	  double* z_cmd_;
	  double* a_cmd_;
	  double* b_cmd_;
	  double* c_cmd_;

	};

	class KukaCartesianStiffnessStateInterface : public HardwareResourceManager<KukaCartesianStiffnessStateHandle> {};

	class KUKACartesianStiffnessCommandInterface : public HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources> {};

	class KUKACartesianStiffnessInterface : public KUKACartesianStiffnessCommandInterface {};
	
	
	class KukaCartesianDampingStateInterface : public HardwareResourceManager<KukaCartesianDampingStateHandle> {};

	class KUKACartesianDampingCommandInterface : public HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources> {};

	class KUKACartesianDampingInterface : public KUKACartesianDampingCommandInterface {};
	
	
	
	 /** \brief Hardware interface to support commanding KUKA LWR 4+ cartesian.
	 *
	 *
	 * \note Getting a cartesian variable handle through the getHandle() method \e will claim that resource.
	 *
	 */
	class KUKACartesianCommandInterface : 	public HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>, 
											public HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>,
											public HardwareResourceManager<JointHandle, ClaimResources>
	{
	public:
		/// these "using" directives are needed to disambiguate
		using HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>::ResourceManager<KUKACartesianStiffnessHandle>::registerHandle;
		using HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>::ResourceManager<KUKACartesianDampingHandle>::registerHandle;
		using HardwareResourceManager<JointHandle, ClaimResources>::ResourceManager<JointHandle>::registerHandle;
		
		/// getHandle needs to be discriminated as there is no way of deducing which functions to call (only differ based on return type)
		/// unless using a Proxy class, and exploiting the cast operator
		class handleProxy
		{
			KUKACartesianCommandInterface* myOwner;
			const std::string& myName;
			public:
				handleProxy( KUKACartesianCommandInterface* owner, const std::string& name ) : myOwner(owner), myName(name) {}
				/// the commented implementation is more generic, and more error prone: could try to call also different cast,
				/// and may thus result in inconsistencies (as HardwareResourceManager<T,ClaimResources> only works for some T's
				// template<class T>
				// operator T() const
				// {
				//     return myOwner->HardwareResourceManager<T, ClaimResources>::getHandle(myName);
				// }
				operator KUKACartesianStiffnessHandle() const
				{
					return myOwner->HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>::getHandle(myName);
				}
				operator KUKACartesianDampingHandle() const
				{
					return myOwner->HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>::getHandle(myName);
				}
				operator JointHandle() const
				{
					return myOwner->HardwareResourceManager<JointHandle, ClaimResources>::getHandle(myName);
				}
		};
		
		handleProxy getHandle(const std::string& name)
		{
			return handleProxy(this, name);
		}
		
		/// get names for all resources
		std::vector<std::string> getNames() const
		{
			std::vector<std::string> vect_joint_names = this->HardwareResourceManager<JointHandle, ClaimResources>::getNames();
			std::vector<std::string> vect_cart_siff_names = this->HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>::getNames();
			std::vector<std::string> vect_cart_damp_names = this->HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>::getNames();
			
			
			vect_joint_names.insert(vect_joint_names.end(), std::make_move_iterator(vect_cart_siff_names.begin()), std::make_move_iterator(vect_cart_siff_names.end()));
			vect_joint_names.insert(vect_joint_names.end(), std::make_move_iterator(vect_cart_damp_names.begin()), std::make_move_iterator(vect_cart_damp_names.end()));
			
			return vect_joint_names;
		}
		
		/// Clear the resources this interface is claiming
		void clearClaims()
		{
			this->HardwareResourceManager<JointHandle, ClaimResources>::clearClaims();
			this->HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>::clearClaims();
			this->HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>::clearClaims();
			return;
		}
		
		/// Get the list of resources this interface is currently claiming
		std::set<std::string> getClaims() const
		{
			std::set<std::string> set_joint_claims = this->HardwareResourceManager<JointHandle, ClaimResources>::getClaims();
			std::set<std::string> set_stiff_claims = this->HardwareResourceManager<KUKACartesianStiffnessHandle, ClaimResources>::getClaims();
			std::set<std::string> set_damp_claims = this->HardwareResourceManager<KUKACartesianDampingHandle, ClaimResources>::getClaims();
			
			set_joint_claims.insert(set_stiff_claims.begin(), set_stiff_claims.end());
			set_joint_claims.insert(set_damp_claims.begin(), set_damp_claims.end());
			
			return set_joint_claims;
		}
	};
	
	/// \ref KUKACartesianInterface for commanding cartesian-based KUKA LWR 4+.
	class KUKACartesianInterface : public KUKACartesianCommandInterface {};

}

#endif
