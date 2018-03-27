#include "rqt_plugins/rqt_platform_sigma_controller_manager.h"
#include <pluginlib/class_list_macros.h>

#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>

namespace platform_sigma_plugins_ns {
	
	ControllerManagerPlugin::ControllerManagerPlugin()
	: rqt_gui_cpp::Plugin(), widget_(0)
	{
		setObjectName("Plugin Controller Manager");
	}

	void ControllerManagerPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
	{
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
	}
	
	void ControllerManagerPlugin::tree_controllers_widget_ContextMenu(const QPoint& aPoint)
	{
		
	}

	void ControllerManagerPlugin::shutdownPlugin()
	{
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

} // End of namespace

PLUGINLIB_DECLARE_CLASS(platform_sigma_plugins_ns, ControllerManagerPlugin, platform_sigma_plugins_ns::ControllerManagerPlugin, rqt_gui_cpp::Plugin)

