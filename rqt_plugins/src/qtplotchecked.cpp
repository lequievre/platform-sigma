/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include "rqt_plugins/qtplotchecked.h"
#include <algorithm>
	
namespace platform_sigma_plugins_ns {

	QtPlotChecked::QtPlotChecked( QWidget *parent, const QString& title_plot, const QString& axis_title_left, const QString& axis_title_bottom, const QPair<double, double>& axis_left_scales )
        : QWidget(parent), title_plot_(title_plot), axis_title_left_(axis_title_left), axis_title_bottom_(axis_title_bottom), axis_left_scales_(axis_left_scales)
    {
		setVectCurveColor_();
			
		plot_ = new QwtPlot();
		plot_->setCanvasBackground(Qt::white);
		plot_->setTitle(title_plot_);
		plot_->setAxisTitle(QwtPlot::yLeft, axis_title_left_);
		plot_->setAxisTitle(QwtPlot::xBottom, axis_title_bottom_);
		plot_->setAxisScale(QwtPlot::yLeft, axis_left_scales_.first, axis_left_scales_.second);
		
		plot_grid_ = new QwtPlotGrid();
		plot_grid_->setPen(QPen(QColor(196,196,196)));
		plot_grid_->attach(plot_);
		
		plot_legend_ = new QwtLegend();
		plot_->insertLegend(plot_legend_, QwtPlot::BottomLegend);
		
		QString curve_name;
		
		curve_size_ = 4;
		
		for (size_t i=0; i<7; i++)
		{
			map_curve_[i] = new QwtPlotCurve(QString("Joint%1").arg(i));
			
			map_data_curve_[i].resize(50);
			vect_time_curve_.resize(50);
			
			map_curve_[i]->setPen(QPen(QColor(vect_curve_color_[i]), curve_size_, Qt::SolidLine));
			map_curve_[i]->setRawSamples(vect_time_curve_.data(), map_data_curve_[i].data(), 50);
			map_curve_[i]->attach(plot_);
			
			//QwtSymbol * aCurveSymbol = new QwtSymbol(QwtSymbol::Ellipse, QBrush(color_curve), QPen(Qt::black), QSize(4,4));
			//map_curve_symbol_.insert(curve_name,aCurveSymbol);
		}
		
		plot_->show();
		plot_->replot();
		
		vlayout_global_ = new QVBoxLayout();
		
		vlayout_global_->addWidget(plot_);
		
		setLayout(vlayout_global_);
    }
    
    void QtPlotChecked::setVectCurveColor_()
    {
		vect_curve_color_.clear();
		vect_curve_color_.append(Qt::black);
		vect_curve_color_.append(Qt::green);
		vect_curve_color_.append(Qt::red);
		vect_curve_color_.append(Qt::yellow);
		vect_curve_color_.append(Qt::blue);
		vect_curve_color_.append(Qt::cyan);
		vect_curve_color_.append(Qt::gray);
	}

	void QtPlotChecked::updateDataCurves(QVector<double> values, double timeDuration)
	{
		
		
		if (vect_time_curve_.size() > 49)
		{
			vect_time_curve_.remove(0);
			
			for (size_t i=0; i<7; i++)
			{
				map_data_curve_[i].remove(0);
			}
		}
		
		vect_time_curve_.append(timeDuration);
		
		for (size_t i=0; i<7; i++)
		{				
			map_data_curve_[i].append(values[i]);	
		}
	}
	
	void QtPlotChecked::updateAxisScale()
	{
		plot_->setAxisScale(QwtPlot::xBottom, vect_time_curve_[0], vect_time_curve_[vect_time_curve_.size()-1]);
		
		plot_->replot();
	}
    
} // End of namespace
