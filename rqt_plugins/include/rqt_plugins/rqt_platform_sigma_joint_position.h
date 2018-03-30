/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#ifndef joint_position_plugin_H
#define joint_position_plugin_H

#include "ros/ros.h"

#include <rqt_gui_cpp/plugin.h>

// Qt graphics
#include <QtGui/QWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QLineEdit>

// Qwt graphics
#include <qwt_slider.h>
#include <qwt_text_label.h>


// ROS msgs
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#define TRACE_JointPositionPlugin_ACTIVATED 0

namespace platform_sigma_plugins_ns {
	
	class JointPositionPlugin : public rqt_gui_cpp::Plugin
	{
		Q_OBJECT
		
		public:

			JointPositionPlugin();

			virtual void initPlugin(qt_gui_cpp::PluginContext& context);

			virtual void shutdownPlugin();

			virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, 
									  qt_gui_cpp::Settings& instance_settings) const;

			virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, 
										 const qt_gui_cpp::Settings& instance_settings);
		 public slots:
		 
			void setValueLineJ0(double value);
			void setValueLineJ1(double value);
			void setValueLineJ2(double value);
			void setValueLineJ3(double value);
			void setValueLineJ4(double value);
			void setValueLineJ5(double value);
			void setValueLineJ6(double value);
			
			void updateValueSliderJ0();
			void updateValueSliderJ1();
			void updateValueSliderJ2();
			void updateValueSliderJ3();
			void updateValueSliderJ4();
			void updateValueSliderJ5();
			void updateValueSliderJ6();
			
			void resetPosition();
			void sendPosition();
			
			void doUpdateLabelJs0(double position);
			void doUpdateLabelJs1(double position);
			void doUpdateLabelJs2(double position);
			void doUpdateLabelJs3(double position);
			void doUpdateLabelJs4(double position);
			void doUpdateLabelJs5(double position);
			void doUpdateLabelJs6(double position);
			
		  signals:
		  
				void updateLabelJs0(double position);
				void updateLabelJs1(double position);
				void updateLabelJs2(double position);
				void updateLabelJs3(double position);
				void updateLabelJs4(double position);
				void updateLabelJs5(double position);
				void updateLabelJs6(double position);
		 
		  private:
			QWidget* widget_;
			QVBoxLayout* vlayout_outer_;
			QHBoxLayout* hlayout_j0_, *hlayout_j1_, *hlayout_j2_, *hlayout_j3_, *hlayout_j4_, *hlayout_j5_, *hlayout_j6_, *hlayout_buttons_;
			QLabel* label_j0_, *label_j1_, *label_j2_, *label_j3_, *label_j4_, *label_j5_, *label_j6_;
			QwtSlider* slider_j0_, *slider_j1_, *slider_j2_, *slider_j3_, *slider_j4_, *slider_j5_, *slider_j6_;
			QwtTextLabel* label_js_j0_, * label_js_j1_,* label_js_j2_,* label_js_j3_,* label_js_j4_,* label_js_j5_,* label_js_j6_;
			QLineEdit* line_j0_, *line_j1_, *line_j2_, *line_j3_, *line_j4_, *line_j5_, *line_j6_;
			QPushButton* button_send_, *button_reset_;
			
			/* Publishers */
			ros::Publisher  pub_send_joint_position_;
			ros::Subscriber sub_joint_handle;
			
			ros::Time current_time, previous_time;
			
			/* Ros msg */
			std_msgs::Float64MultiArray joint_position_msg_;
			
			void setupROSComponents_();
			void shutdownROSComponents_();
			
			void jsCallback_(const sensor_msgs::JointState::ConstPtr& msg);
			
		
	}; // End of class
	
} // End of namespace

#endif
