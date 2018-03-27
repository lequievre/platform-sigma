#include "rqt_plugins/rqt_platform_sigma_controller_manager.h"
#include <pluginlib/class_list_macros.h>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
// cf /opt/ros/indigo/include/controller_manager_msgs

namespace platform_sigma_plugins_ns {
	
	ControllerManagerPlugin::ControllerManagerPlugin()
	: rqt_gui_cpp::Plugin(), widget_(0)
	{
		setObjectName("Plugin Controller Manager");
	}

	void ControllerManagerPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		
		setupROSComponents_();
		
		// create a main widget named widget_
		widget_ = new QWidget();
		widget_->setWindowTitle("Controller Manager");
		
		// create layouts
        vlayout_outer_ = new QVBoxLayout();
        vlayout_outer_->setObjectName("vertical_layout_outer");
        
        hlayout_top_ = new QHBoxLayout();
        hlayout_top_->setObjectName("horizontal_layout_top");
        
		vlayout_outer_->addLayout(hlayout_top_);
		
		
		// create content of horizontal layout
		// create a label
		ns_label_ = new QLabel();
        ns_label_->setObjectName("ns_label_");
        ns_label_->setText("Kuka Namespace :");
        QSizePolicy fixed_policy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        ns_label_->setSizePolicy(fixed_policy);
        hlayout_top_->addWidget(ns_label_);
        // create a combo box for kuka namespaces
        ns_combo_ = new QComboBox();
        ns_combo_->setObjectName("ns_combo_");
        ns_combo_->addItem("kuka_lwr_left");
        ns_combo_->addItem("kuka_lwr_right");
		hlayout_top_->addWidget(ns_combo_);
		
		// create tree/list widget of kuka controllers
		tree_controllers_widget_ = new QTreeWidget();
		tree_controllers_widget_ ->setObjectName("tree_controllers_widget_");
		
		column_names_list_ << "name" << "state" << "type" << "hw_iface" << "resources";
		column_names_pretty_list_ << "Controller Name" << "State" << "Type" << "HW Interface" << "Claimed Resources";
		
		tree_controllers_widget_->setColumnCount(column_names_list_.size());
		tree_controllers_widget_->setHeaderLabels(column_names_pretty_list_);
		tree_controllers_widget_->sortByColumn(0, Qt::AscendingOrder);
		tree_controllers_widget_->setContextMenuPolicy(Qt::CustomContextMenu);
		
		connect(tree_controllers_widget_, SIGNAL( customContextMenuRequested( const QPoint& ) ),
             this, SLOT( tree_controllers_widget_ContextMenu( const QPoint& ) ) );
             
		vlayout_outer_->addWidget(tree_controllers_widget_);
		
		// set widget_ to main widget
		widget_->setLayout(vlayout_outer_);
		context.addWidget(widget_);
		
		updateListControllers_();
	}
	
	void ControllerManagerPlugin::tree_controllers_widget_ContextMenu(const QPoint& aPoint)
	{
		
	}

	void ControllerManagerPlugin::shutdownPlugin()
	{
		shutdownROSComponents_();
	}

	void ControllerManagerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
								qt_gui_cpp::Settings& instance_settings) const
	{
		// TODO save intrinsic configuration, usually using:
		// instance_settings.setValue(k, v)
	}

	void ControllerManagerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
								   const qt_gui_cpp::Settings& instance_settings)
	{
		// TODO restore intrinsic configuration, usually using:
		// v = instance_settings.value(k)
	}
	
	void ControllerManagerPlugin::setupROSComponents_()
	{
		map_list_service_client_.insert("kuka_lwr_left",getNodeHandle().serviceClient<controller_manager_msgs::ListControllers>("/kuka_lwr_left/controller_manager/list_controllers")); 
		map_list_service_client_.insert("kuka_lwr_right",getNodeHandle().serviceClient<controller_manager_msgs::ListControllers>("/kuka_lwr_right/controller_manager/list_controllers")); 
	
		map_switch_service_client_.insert("kuka_lwr_left",getNodeHandle().serviceClient<controller_manager_msgs::SwitchController>("/kuka_lwr_left/controller_manager/switch_controller")); 
		map_switch_service_client_.insert("kuka_lwr_right",getNodeHandle().serviceClient<controller_manager_msgs::SwitchController>("/kuka_lwr_right/controller_manager/switch_controller"));	
	}
	
	void ControllerManagerPlugin::shutdownROSComponents_()
	{
		map_list_service_client_["kuka_lwr_left"].shutdown();
		map_list_service_client_["kuka_lwr_right"].shutdown();
		
		map_switch_service_client_["kuka_lwr_left"].shutdown();
		map_switch_service_client_["kuka_lwr_right"].shutdown();
	}
	
	void ControllerManagerPlugin::updateListControllers_()
	{
		
		ros::ServiceClient controller_list_client = map_list_service_client_[ns_combo_->currentText()];

		controller_manager_msgs::ListControllers controller_list;
	
		controller_list_client.call(controller_list);
	
		for (unsigned int i=0;i<controller_list.response.controller.size() ;i++ )
		{
			//tree_controllers_widget_->clear();
			
			// search by name at column 0
			QList<QTreeWidgetItem*> items_found = tree_controllers_widget_->findItems(controller_list.response.controller[i].name.c_str(), Qt::MatchExactly, 0);
			if (items_found.count() == 0)
			{
				// Create a new item
				QTreeWidgetItem* new_item = new QTreeWidgetItem(tree_controllers_widget_);
                new_item->setText(0, controller_list.response.controller[i].name.c_str());
			}
			else
			{
				// modify an item
			}
			
		//std::cout << "name = " << controller_list.response.controller[i].name << ", type = " << controller_list.response.controller[i].type << ", hw interface = " << controller_list.response.controller[i].hardware_interface << ", state = " << controller_list.response.controller[i].state << std::endl;
		/*std::cout << "Ressources :" << std::endl;
		for (unsigned int j=0;j<controller_list.response.controller[i].resources.size() ;j++ )
		{
			std::cout << "-> " << controller_list.response.controller[i].resources[j] << std::endl;
		}*/
		}
	}

} // End of namespace

PLUGINLIB_DECLARE_CLASS(platform_sigma_plugins_ns, ControllerManagerPlugin, platform_sigma_plugins_ns::ControllerManagerPlugin, rqt_gui_cpp::Plugin)

