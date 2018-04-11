/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include "rqt_plugins/qtplotchecked.h"
	
namespace platform_sigma_plugins_ns {

	QtPlotChecked::QtPlotChecked( QWidget *parent, const QString& title_plot, const QString& axis_title_left, const QString& axis_title_bottom, const QPair<double, double>& axis_left_scales )
        : QWidget(parent), title_plot_(title_plot), axis_title_left_(axis_title_left), axis_title_bottom_(axis_title_bottom), axis_left_scales_(axis_left_scales)
    {
    
    }
    
} // End of namespace
