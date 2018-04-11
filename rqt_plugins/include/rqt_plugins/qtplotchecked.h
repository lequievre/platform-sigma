/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#ifndef Qt_Plot_Checked_Widget_H
#define Qt_Plot_Checked_Widget_H

// Qt graphics
#include <QtGui/QWidget>
#include <QtCore/QPair>

namespace platform_sigma_plugins_ns {
	
	class QtPlotChecked : public QWidget
	{
		Q_OBJECT
		
		public:
			QtPlotChecked( QWidget *parent = 0, const QString& title_plot =  QString(), const QString& axis_title_left =  QString(), const QString& axis_title_bottom =  QString(), const QPair<double, double>& axis_left_scales = QPair<double,double>(0.0,0.0) );
			~QtPlotChecked() {}
			
		private:
			QPair<double,double> axis_left_scales_;
			QString title_plot_;
			QString axis_title_left_, axis_title_bottom_;
		
	}; // End of class
	
} // End of namespace
    
#endif 
