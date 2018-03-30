/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include "rqt_plugins/rqt_platform_sigma_joint_position.h"
#include <pluginlib/class_list_macros.h>

#include <math.h>

namespace platform_sigma_plugins_ns {
	
	JointPositionPlugin::JointPositionPlugin()
	: rqt_gui_cpp::Plugin(), widget_(0), hlayout_j0_(0), hlayout_j1_(0), hlayout_j2_(0), hlayout_j3_(0), hlayout_j4_(0), hlayout_j5_(0), hlayout_j6_(0), 
	 label_j0_(0), label_j1_(0), label_j2_(0), label_j3_(0), label_j4_(0), label_j5_(0), label_j6_(0), 
	 button_send_(0), button_reset_(0),
	 slider_j0_(0), slider_j1_(0), slider_j2_(0), slider_j3_(0), slider_j4_(0), slider_j5_(0), slider_j6_(0),
	 line_j0_(0), line_j1_(0), line_j2_(0), line_j3_(0), line_j4_(0), line_j5_(0), line_j6_(0)
	{
		setObjectName("Plugin Joint Position");
	}
	
	void JointPositionPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		setupROSComponents_();
		
		// create a main widget named widget_
		widget_ = new QWidget();
		widget_->setWindowTitle("Joint Position");
	
		// create layouts
		vlayout_outer_ = new QVBoxLayout();
		vlayout_outer_->setObjectName("vertical_layout_outer");
		
		QSizePolicy fixed_policy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		
		// start j0
		hlayout_j0_ = new QHBoxLayout();
        hlayout_j0_->setObjectName("horizontal_layout_j0");
        
        label_j0_ = new QLabel();
        label_j0_->setObjectName("label_j0_");
        label_j0_->setText("Joint 0 :");
        label_j0_->setSizePolicy(fixed_policy);
        hlayout_j0_->addWidget(label_j0_);
        
        slider_j0_ = new QwtSlider(widget_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j0_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j0_->setValue( 0 );
		connect( slider_j0_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ0(double)) );
		hlayout_j0_->addWidget(slider_j0_);
		
		line_j0_ = new QLineEdit();
		line_j0_->setObjectName("line_j0_");
		line_j0_->setSizePolicy(fixed_policy);
		line_j0_->setText(QString::number(slider_j0_->value() ));
		connect( line_j0_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ0()) );
		
		hlayout_j0_->addWidget(line_j0_);
		
		vlayout_outer_->addLayout(hlayout_j0_);
		// end j0
		
		// start j1
		hlayout_j1_ = new QHBoxLayout();
        hlayout_j1_->setObjectName("horizontal_layout_j1");
        
        label_j1_ = new QLabel();
        label_j1_->setObjectName("label_j1_");
        label_j1_->setText("Joint 1 :");
        label_j1_->setSizePolicy(fixed_policy);
        hlayout_j1_->addWidget(label_j1_);
        
        slider_j1_ = new QwtSlider(widget_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j1_->setRange((-120 * M_PI / 180), (120 * M_PI / 180), 0.1, 1);
		slider_j1_->setValue( 0 );
		connect( slider_j1_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ1(double)) );
		hlayout_j1_->addWidget(slider_j1_);
		
		line_j1_ = new QLineEdit();
		line_j1_->setObjectName("line_j1_");
		line_j1_->setSizePolicy(fixed_policy);
		line_j1_->setText(QString::number(slider_j1_->value() ));
		connect( line_j1_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ1()) );
		
		hlayout_j1_->addWidget(line_j1_);
		
		vlayout_outer_->addLayout(hlayout_j1_);
		// end j1
		
		// start j2
		hlayout_j2_ = new QHBoxLayout();
        hlayout_j2_->setObjectName("horizontal_layout_j2");
        
        label_j2_ = new QLabel();
        label_j2_->setObjectName("label_j2_");
        label_j2_->setText("Joint 2 :");
        label_j2_->setSizePolicy(fixed_policy);
        hlayout_j2_->addWidget(label_j2_);
        
        slider_j2_ = new QwtSlider(widget_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j2_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j2_->setValue( 0 );
		connect( slider_j2_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ2(double)) );
		hlayout_j2_->addWidget(slider_j2_);
		
		line_j2_ = new QLineEdit();
		line_j2_->setObjectName("line_j2_");
		line_j2_->setSizePolicy(fixed_policy);
		line_j2_->setText(QString::number(slider_j2_->value() ));
		connect( line_j2_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ2()) );
		
		hlayout_j2_->addWidget(line_j2_);
		
		vlayout_outer_->addLayout(hlayout_j2_);
		// end j2
		
		// start j3
		hlayout_j3_ = new QHBoxLayout();
        hlayout_j3_->setObjectName("horizontal_layout_j3");
        
        label_j3_ = new QLabel();
        label_j3_->setObjectName("label_j3_");
        label_j3_->setText("Joint 3 :");
        label_j3_->setSizePolicy(fixed_policy);
        hlayout_j3_->addWidget(label_j3_);
        
        slider_j3_ = new QwtSlider(widget_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j3_->setRange((-120 * M_PI / 180), (120 * M_PI / 180), 0.1, 1);
		slider_j3_->setValue( 0 );
		connect( slider_j3_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ3(double)) );
		hlayout_j3_->addWidget(slider_j3_);
		
		line_j3_ = new QLineEdit();
		line_j3_->setObjectName("line_j3_");
		line_j3_->setSizePolicy(fixed_policy);
		line_j3_->setText(QString::number(slider_j3_->value() ));
		connect( line_j3_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ3()) );
		
		hlayout_j3_->addWidget(line_j3_);
		
		vlayout_outer_->addLayout(hlayout_j3_);
		// end j3
		
		// start j4
		hlayout_j4_ = new QHBoxLayout();
        hlayout_j4_->setObjectName("horizontal_layout_j4");
        
        label_j4_ = new QLabel();
        label_j4_->setObjectName("label_j4_");
        label_j4_->setText("Joint 4 :");
        label_j4_->setSizePolicy(fixed_policy);
        hlayout_j4_->addWidget(label_j4_);
        
        slider_j4_ = new QwtSlider(widget_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j4_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j4_->setValue( 0 );
		connect( slider_j4_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ4(double)) );
		hlayout_j4_->addWidget(slider_j4_);
		
		line_j4_ = new QLineEdit();
		line_j4_->setObjectName("line_j4_");
		line_j4_->setSizePolicy(fixed_policy);
		line_j4_->setText(QString::number(slider_j4_->value() ));
		connect( line_j4_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ4()) );
		
		hlayout_j4_->addWidget(line_j4_);
		
		vlayout_outer_->addLayout(hlayout_j4_);
		// end j4
		
		// start j5
		hlayout_j5_ = new QHBoxLayout();
        hlayout_j5_->setObjectName("horizontal_layout_j5");
        
        label_j5_ = new QLabel();
        label_j5_->setObjectName("label_j5_");
        label_j5_->setText("Joint 5 :");
        label_j5_->setSizePolicy(fixed_policy);
        hlayout_j5_->addWidget(label_j5_);
        
        slider_j5_ = new QwtSlider(widget_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j5_->setRange((-120 * M_PI / 180), (120 * M_PI / 180), 0.1, 1);
		slider_j5_->setValue( 0 );
		connect( slider_j5_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ5(double)) );
		hlayout_j5_->addWidget(slider_j5_);
		
		line_j5_ = new QLineEdit();
		line_j5_->setObjectName("line_j5_");
		line_j5_->setSizePolicy(fixed_policy);
		line_j5_->setText(QString::number(slider_j5_->value() ));
		connect( line_j5_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ5()) );
		
		hlayout_j5_->addWidget(line_j5_);
		
		vlayout_outer_->addLayout(hlayout_j5_);
		// end j5
		
		// start j6
		hlayout_j6_ = new QHBoxLayout();
        hlayout_j6_->setObjectName("horizontal_layout_j6");
        
        label_j6_ = new QLabel();
        label_j6_->setObjectName("label_j6_");
        label_j6_->setText("Joint 6 :");
        label_j6_->setSizePolicy(fixed_policy);
        hlayout_j6_->addWidget(label_j6_);
        
        slider_j6_ = new QwtSlider(widget_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j6_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j6_->setValue( 0 );
		connect( slider_j6_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ6(double)) );
		hlayout_j6_->addWidget(slider_j6_);
		
		line_j6_ = new QLineEdit();
		line_j6_->setObjectName("line_j6_");
		line_j6_->setSizePolicy(fixed_policy);
		line_j6_->setText(QString::number(slider_j6_->value() ));
		connect( line_j6_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ6()) );
		
		hlayout_j6_->addWidget(line_j6_);
		
		vlayout_outer_->addLayout(hlayout_j6_);
		// end j6
		
		// buttons send and reset
		hlayout_buttons_ = new QHBoxLayout();
		hlayout_buttons_->setObjectName("horizontal_layout_buttons");
		
		button_reset_ = new QPushButton("Reset");
		connect(button_reset_, SIGNAL(pressed()), this, SLOT(resetPosition()));
		hlayout_buttons_->addWidget(button_reset_);
		
		button_send_ = new QPushButton("Send");
		connect(button_send_, SIGNAL(pressed()), this, SLOT(sendPosition()));
		hlayout_buttons_->addWidget(button_send_);
		
		vlayout_outer_->addLayout(hlayout_buttons_);
		
		// set widget_ to main widget
		widget_->setLayout(vlayout_outer_);
		context.addWidget(widget_);
	}
	
	void JointPositionPlugin::sendPosition()
	{
		std::vector<double> vect;
		vect.resize(7);
		
		vect[0] = slider_j0_->value();
		
				
		joint_position_msg_.layout.dim.clear();
		joint_position_msg_.layout.dim.push_back(std_msgs::MultiArrayDimension());
		joint_position_msg_.layout.dim[0].size = vect.size();
		joint_position_msg_.layout.dim[0].stride = 1;
		joint_position_msg_.layout.dim[0].label = "x_values"; // or whatever name you typically use to index vec1

		// copy in the data
		joint_position_msg_.data.clear();
		joint_position_msg_.data.insert(joint_position_msg_.data.end(), vect.begin(), vect.end());
		
		pub_send_joint_position_.publish(joint_position_msg_);
	}
	
	void JointPositionPlugin::resetPosition()
	{
		
	}
	
	void JointPositionPlugin::setValueLineJ0(double value)
	{
		line_j0_->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ0()
	{
		slider_j0_->setValue(line_j0_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ1(double value)
	{
		line_j1_->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ1()
	{
		slider_j1_->setValue(line_j1_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ2(double value)
	{
		line_j2_->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ2()
	{
		slider_j2_->setValue(line_j2_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ3(double value)
	{
		line_j3_->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ3()
	{
		slider_j3_->setValue(line_j3_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ4(double value)
	{
		line_j4_->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ4()
	{
		slider_j4_->setValue(line_j4_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ5(double value)
	{
		line_j5_->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ5()
	{
		slider_j5_->setValue(line_j5_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ6(double value)
	{
		line_j6_->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ6()
	{
		slider_j6_->setValue(line_j6_->text().toDouble());
	}
	
	
	void JointPositionPlugin::shutdownPlugin()
	{
		shutdownROSComponents_();
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
	
	void JointPositionPlugin::setupROSComponents_()
	{
		/* Setup publishers */
		pub_send_joint_position_ = getNodeHandle().advertise<std_msgs::Float64MultiArray>("/kuka_lwr_left/kuka_group_command_controller_fri/command", 1);
	}
	
	
	void JointPositionPlugin::shutdownROSComponents_()
	{
		
	}

	
} // End of namespace

PLUGINLIB_DECLARE_CLASS(platform_sigma_plugins_ns, JointPositionPlugin, platform_sigma_plugins_ns::JointPositionPlugin, rqt_gui_cpp::Plugin)


