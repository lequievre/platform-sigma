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
#include <QtGui/QComboBox>
#include <QtGui/QTabWidget>
#include <QtGui/QTableWidget>

// Qwt graphics
#include <qwt_slider.h>
#include <qwt_text_label.h>

// ROS msgs
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#define TRACE_JointPositionPlugin_ACTIVATED 1

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
			
			void sendPosition();
			
			void doUpdateLabelJs(QVector<double> positions);
			
			void ns_combo_changed(int);
			void resetSlidersPositions();
			
		  signals:
				
			void updateLabelJs(QVector<double> positions);
		 
		  private:
		  
			QTabWidget* tab_widget_;
			QTableWidget* table_widget_global_;
			QWidget* widget_sliders_;
			
			QVBoxLayout* vlayout_global_;
			QHBoxLayout* hlayout_ns_;
			
			QwtSlider* slider_j0_, *slider_j1_, *slider_j2_, *slider_j3_, *slider_j4_, *slider_j5_, *slider_j6_;
			QLineEdit* line_j0_, *line_j1_, *line_j2_, *line_j3_, *line_j4_, *line_j5_, *line_j6_;
			QPushButton* button_send_, *button_reset_;
			
			QComboBox* ns_combo_;
			
			/* Publishers && Subscribers */
			QMap<QString, ros::Publisher> map_pub_joint_position_;
			QMap<QString, ros::Subscriber> map_sub_joint_handle_;
			
			QMap<QString, QVector<double> > map_selected_joint_values_;
			QMap<QString, QVector<double> > map_current_joint_state_values_;
			QMap<QString, bool > map_sliders_is_init_;
			
			/* Ros msg */
			std_msgs::Float64MultiArray joint_position_msg_;
			
			void setupROSComponents_();
			void shutdownROSComponents_();
			
			void jsCallback_left_(const sensor_msgs::JointState::ConstPtr& msg);
			void jsCallback_right_(const sensor_msgs::JointState::ConstPtr& msg);
			
		
	}; // End of class
	
} // End of namespace

#endif
