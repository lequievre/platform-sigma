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
#include <QtGui/QWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QLineEdit>

#include <qwt_slider.h>

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
			void updateValueSliderJ0();
		 
		  private:
			QWidget* widget_;
			QVBoxLayout* vlayout_outer_;
			QHBoxLayout* hlayout_j0_, *hlayout_j1_, *hlayout_j2_, *hlayout_j3_, *hlayout_j4_, *hlayout_j5_, *hlayout_j6_;
			QLabel* label_j0_, *label_j1_, *label_j2_, *label_j3_, *label_j4_, *label_j5_, *label_j6_;
			QwtSlider* slider_j0_, *slider_j1_, *slider_j2_, *slider_j3_, *slider_j4_, *slider_j5_, *slider_j6_;
			QLineEdit* line_j0_, *line_j1_, *line_j2_, *line_j3_, *line_j4_, *line_j5_, *line_j6_;
			QPushButton* button_send_, *button_reset_;
			
		
	}; // End of class
	
} // End of namespace

#endif
