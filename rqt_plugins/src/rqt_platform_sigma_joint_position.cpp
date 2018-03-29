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
	: rqt_gui_cpp::Plugin(), widget_(0), hlayout_j0_(0), hlayout_j1_(0), hlayout_j2_(0), hlayout_j3_(0), hlayout_j4_(0),
	 hlayout_j5_(0), hlayout_j6_(0), label_j0_(0), label_j1_(0), label_j2_(0), label_j3_(0), label_j4_(0), 
	 label_j5_(0), label_j6_(0), button_send_(0), button_reset_(0)
	{
		setObjectName("Plugin Joint Position");
	}
	
	void JointPositionPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		// create a main widget named widget_
		widget_ = new QWidget();
		widget_->setWindowTitle("Joint Position");
	
		// create layouts
		vlayout_outer_ = new QVBoxLayout();
		vlayout_outer_->setObjectName("vertical_layout_outer");
		
		QSizePolicy fixed_policy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		
		// j0
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
		
		// set widget_ to main widget
		widget_->setLayout(vlayout_outer_);
		context.addWidget(widget_);
	}
	
	void JointPositionPlugin::setValueLineJ0(double value)
	{
		line_j0_->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ0()
	{
		slider_j0_->setValue(line_j0_->text().toDouble());
	}
	
	void JointPositionPlugin::shutdownPlugin()
	{
		
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
	
} // End of namespace

PLUGINLIB_DECLARE_CLASS(platform_sigma_plugins_ns, JointPositionPlugin, platform_sigma_plugins_ns::JointPositionPlugin, rqt_gui_cpp::Plugin)


