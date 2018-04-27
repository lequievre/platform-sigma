/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include "rqt_plugins/rqt_platform_sigma_joint_position.h"
#include <pluginlib/class_list_macros.h>

#include <controller_manager_msgs/ListControllers.h>
// cf /opt/ros/indigo/include/controller_manager_msgs

#include <math.h>
#include <QtCore/QTextStream>
#include <QtCore/QMetaType>
#include <QtGui/QHeaderView>

namespace platform_sigma_plugins_ns {
	
	JointPositionPlugin::JointPositionPlugin()
	: rqt_gui_cpp::Plugin(), widget_sliders_(0), tab_widget_(0), vlayout_global_(0), button_send_(0), button_reset_(0), firstTime_(0)
	{
		setObjectName("Plugin Joint Position");
		qRegisterMetaType<QVector<double> >("QVector<double>");
	}
	
	void JointPositionPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		
		// create a main widget for sliders position
		widget_sliders_ = new QWidget();
		widget_sliders_->setWindowTitle("Joint Position");
		
		vlayout_global_ = new QVBoxLayout();
		vlayout_global_->setObjectName("vertical_layout_global");
		
		 // create a combo box for kuka namespaces
        ns_combo_ = new QComboBox();
        ns_combo_->setObjectName("ns_combo_");
        ns_combo_->addItem("kuka_lwr_left");
        ns_combo_->addItem("kuka_lwr_right");
		
		connect(ns_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(ns_combo_changed(int)));
		
		vlayout_global_->addWidget(ns_combo_);
		
		position_sliders_ = new QtPositionSliders();
		
		vlayout_global_->addWidget(position_sliders_);
		
		plot_checked_ = new platform_sigma_plugins_ns::QtPlotChecked(widget_sliders_, QString("Movement of the KUKA Joints"), QString("Joint Value (radian)"), QString("Time (sec)"), QPair<double,double>((-170 * M_PI / 180), (170 * M_PI / 180)));
		
		vlayout_global_->addWidget(plot_checked_);
		
		/* End : Plot and Curve specifications */
		
		button_send_ = new QPushButton("Send");
		button_send_->setToolTip("Send sliders positions to ROS position controller");
		connect(button_send_, SIGNAL(pressed()), this, SLOT(sendPosition()));
		vlayout_global_->addWidget(button_send_);
		
		button_reset_ = new QPushButton("Reset");
		button_reset_->setToolTip("Reset sliders position from 'Joint State'");
		connect(button_reset_, SIGNAL(pressed()), this, SLOT(resetSlidersPositions()));
		vlayout_global_->addWidget(button_reset_);
		
		connect(this, SIGNAL(updateLabelJs(QVector<double>)), this, SLOT(doUpdateLabelJs(QVector<double>)));
		
		// set widget_ to main widget
		widget_sliders_->setLayout(vlayout_global_);
		
		tab_widget_ = new QTabWidget();
		tab_widget_->addTab(widget_sliders_,"Sliders Position");
		
		context.addWidget(tab_widget_);
		
		QVector<double> vect_init_joint_values;
		vect_init_joint_values.resize(7);
		vect_init_joint_values.fill(0);
		
		map_selected_joint_values_.insert("kuka_lwr_left",vect_init_joint_values);
		map_selected_joint_values_.insert("kuka_lwr_right",vect_init_joint_values);
		
		map_current_joint_state_values_.insert("kuka_lwr_left",vect_init_joint_values);
		map_current_joint_state_values_.insert("kuka_lwr_right",vect_init_joint_values);
		
		map_sliders_is_init_.insert("kuka_lwr_left",false);
		map_sliders_is_init_.insert("kuka_lwr_right",false);
		
		setupROSComponents_();
		
		timer_ = new QTimer(this);

		// setup signal and slot
		connect(timer_, SIGNAL(timeout()), this, SLOT(doUpdateCurves()));

		// msec
		timer_->start(100);
	}
	
	void JointPositionPlugin::ns_combo_changed(int index)
	{
		resetSlidersPositions();	
	}
	
	void JointPositionPlugin::sendPosition()
	{
		map_selected_joint_values_[ns_combo_->currentText()][0] = position_sliders_->slider_j0_->value();
		map_selected_joint_values_[ns_combo_->currentText()][1] = position_sliders_->slider_j1_->value();
		map_selected_joint_values_[ns_combo_->currentText()][2] = position_sliders_->slider_j2_->value();
		map_selected_joint_values_[ns_combo_->currentText()][3] = position_sliders_->slider_j3_->value();
		map_selected_joint_values_[ns_combo_->currentText()][4] = position_sliders_->slider_j4_->value();
		map_selected_joint_values_[ns_combo_->currentText()][5] = position_sliders_->slider_j5_->value();
		map_selected_joint_values_[ns_combo_->currentText()][6] = position_sliders_->slider_j6_->value();
			
		joint_position_msg_.layout.dim.clear();
		joint_position_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		joint_position_msg_.layout.dim[0].size = map_selected_joint_values_[ns_combo_->currentText()].size();
		joint_position_msg_.layout.dim[0].stride = 1;
		joint_position_msg_.layout.dim[0].label = "x_values"; // or whatever name you typically use to index vec1

		// copy in the data
		joint_position_msg_.data.clear();
		joint_position_msg_.data.insert(joint_position_msg_.data.end(), map_selected_joint_values_[ns_combo_->currentText()].begin(), map_selected_joint_values_[ns_combo_->currentText()].end());
		
		map_pub_joint_position_[ns_combo_->currentText()].publish(joint_position_msg_);
	}
	
	void JointPositionPlugin::resetSlidersPositions()
	{
		position_sliders_->updateSliders(map_current_joint_state_values_[ns_combo_->currentText()]);	
	}
	
	void JointPositionPlugin::shutdownPlugin()
	{
		shutdownROSComponents_();
		
		disconnect(this, SIGNAL(updateLabelJs(QVector<double>)), this, SLOT(doUpdateLabelJs(QVector<double>)));
	
		disconnect(button_send_, SIGNAL(pressed()), this, SLOT(sendPosition()));
		disconnect(button_reset_, SIGNAL(pressed()), this, SLOT(resetSlidersPositions()));
			
		vlayout_global_->removeWidget(ns_combo_);
		vlayout_global_->removeWidget(button_send_);
		vlayout_global_->removeWidget(button_reset_);
			
		tab_widget_->removeTab(0);
			
		if (button_send_)
			delete button_send_;
				
		if (button_reset_)
			delete button_reset_;
			
		if (ns_combo_)
			delete ns_combo_;
			
		if (vlayout_global_)
			delete vlayout_global_;
			
		if (widget_sliders_)
			delete widget_sliders_;
	}
	
	void JointPositionPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
								qt_gui_cpp::Settings& instance_settings) const
	{
		// TODO save intrinsic configuration, usually using:
		// instance_settings.setValue(k, v)
	}

	void JointPositionPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
								   const qt_gui_cpp::Settings& instance_settings)
	{
		// TODO restore intrinsic configuration, usually using:
		// v = instance_settings.value(k)
	}
	
	void JointPositionPlugin::doUpdateCurves()
	{
		plot_checked_->updateAxisScale();
	}
	
	void JointPositionPlugin::jsCallback_left_(const sensor_msgs::JointState::ConstPtr& msg)
	{
		QVector<double> values = QVector<double>::fromStdVector(std::vector<double>(std::begin(msg->position), std::end(msg->position)));
		
		bool zeros = std::all_of(values.begin(), values.end(), [](double position) { return position==0.0; });
		
		if (!zeros)
		{
				map_current_joint_state_values_["kuka_lwr_left"] = values;
		}
		
		if (ns_combo_->currentText() == "kuka_lwr_left")
		{
			if (!zeros)
			{
				double time = msg->header.stamp.sec + (msg->header.stamp.nsec/1e9);
			
				if (firstTime_ == 0)
				{
					firstTime_ = time;
				}

				double timeDuration = time - firstTime_;
			
				plot_checked_->updateDataCurves(values, timeDuration);
				
				if (map_sliders_is_init_["kuka_lwr_left"]==false)
				{
					resetSlidersPositions();
					map_sliders_is_init_["kuka_lwr_left"]=true;
				}
				
				emit updateLabelJs(values);	
			}
		}
	}
	
	void JointPositionPlugin::jsCallback_right_(const sensor_msgs::JointState::ConstPtr& msg)
	{	
		QVector<double> values = QVector<double>::fromStdVector(std::vector<double>(std::begin(msg->position), std::end(msg->position)));
		
		bool zeros = std::all_of(values.begin(), values.end(), [](double position) { return position==0.0; });
		
		if (!zeros)
		{
				map_current_joint_state_values_["kuka_lwr_right"] = values;
		}
		
		if (ns_combo_->currentText() == "kuka_lwr_right")
		{
			if (!zeros)
			{
				double time = msg->header.stamp.sec + (msg->header.stamp.nsec/1e9);
			
				if (firstTime_ == 0)
				{
					firstTime_ = time;
				}

				double timeDuration = time - firstTime_;
			
				plot_checked_->updateDataCurves(values, timeDuration);
				
				if (map_sliders_is_init_["kuka_lwr_right"]==false)
				{
					resetSlidersPositions();
					map_sliders_is_init_["kuka_lwr_right"]=true;
				}
				
				emit updateLabelJs(values);	
			}
		}
	}
	
	void JointPositionPlugin::doUpdateLabelJs(QVector<double> positions)
	{
		position_sliders_->updateLabelJs(positions);
	}
	
	void JointPositionPlugin::setupROSComponents_()
	{
		QString name_of_position_controller = "kuka_group_command_controller_fri";
		
		/* Setup publishers */
		map_pub_joint_position_.insert("kuka_lwr_left",getNodeHandle().advertise<std_msgs::Float64MultiArray>(QString("/kuka_lwr_left/").append(name_of_position_controller).append("/").append("command").toStdString(), 1));
		map_pub_joint_position_.insert("kuka_lwr_right",getNodeHandle().advertise<std_msgs::Float64MultiArray>(QString("/kuka_lwr_right/").append(name_of_position_controller).append("/").append("command").toStdString(), 1));
			
		map_sub_joint_handle_.insert("kuka_lwr_left",getNodeHandle().subscribe(QString("/kuka_lwr_left/").append("joint_states").toStdString(), 100000, &JointPositionPlugin::jsCallback_left_, this));
		map_sub_joint_handle_.insert("kuka_lwr_right",getNodeHandle().subscribe(QString("/kuka_lwr_right/").append("joint_states").toStdString(), 100000, &JointPositionPlugin::jsCallback_right_, this));
	}
	
	void JointPositionPlugin::shutdownROSComponents_()
	{
		map_pub_joint_position_["kuka_lwr_left"].shutdown();
		map_pub_joint_position_["kuka_lwr_right"].shutdown();
			
		map_sub_joint_handle_["kuka_lwr_left"].shutdown();
		map_sub_joint_handle_["kuka_lwr_right"].shutdown();
	}
	
} // End of namespace

PLUGINLIB_DECLARE_CLASS(platform_sigma_plugins_ns, JointPositionPlugin, platform_sigma_plugins_ns::JointPositionPlugin, rqt_gui_cpp::Plugin)


