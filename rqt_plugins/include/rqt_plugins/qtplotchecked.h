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
#include <QtCore/QMap>
#include <QtGui/QVBoxLayout>

// Qwt graphics
#include <qwt_plot_grid.h>
#include <qwt_legend.h>
#include <qwt_symbol.h>

#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_curve.h>

namespace platform_sigma_plugins_ns {
	
	class QtPlotChecked : public QWidget
	{
		Q_OBJECT
		
		public:
			QtPlotChecked( QWidget *parent = 0, const QString& title_plot =  QString(), const QString& axis_title_left =  QString(), const QString& axis_title_bottom =  QString(), const QPair<double, double>& axis_left_scales = QPair<double,double>(0.0,0.0) );
			~QtPlotChecked() {}
			
			void updateDataCurves(QVector<double> values, double timeDuration);
			void updateAxisScale(); 
			
		private:
			QPair<double,double> axis_left_scales_;
			QString title_plot_;
			QString axis_title_left_, axis_title_bottom_;
			
			QMap<int,QVector<double> > map_data_curve_;
			QMap<int,QVector<double> > map_time_curve_;
			QVector<double> vect_time_curve_;
			
			QMap<int,QwtPlotCurve *> map_curve_;
			//QMap<QString,QwtSymbol*> map_curve_symbol_;
			
			QwtPlot      	*plot_;
			QwtPlotGrid 	*plot_grid_;
			QwtLegend		*plot_legend_;
			
			QVBoxLayout* vlayout_global_;
			QVector<QColor> vect_curve_color_;
			int curve_size_;
			
			void setVectCurveColor_();
			
		
	}; // End of class
	
} // End of namespace
    
#endif 
