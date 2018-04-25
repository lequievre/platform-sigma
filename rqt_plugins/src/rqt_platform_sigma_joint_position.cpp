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
	: rqt_gui_cpp::Plugin(), widget_sliders_(0), tab_widget_(0), vlayout_global_(0), table_widget_global_(0),
	 button_send_(0), button_reset_(0),
	 slider_j0_(0), slider_j1_(0), slider_j2_(0), slider_j3_(0), slider_j4_(0), slider_j5_(0), slider_j6_(0),
	 line_j0_(0), line_j1_(0), line_j2_(0), line_j3_(0), line_j4_(0), line_j5_(0), line_j6_(0), firstTime_(0)
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
		
		QSizePolicy fixed_policy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		
		 // create a combo box for kuka namespaces
        ns_combo_ = new QComboBox();
        ns_combo_->setObjectName("ns_combo_");
        ns_combo_->addItem("kuka_lwr_left");
        ns_combo_->addItem("kuka_lwr_right");
		
		connect(ns_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(ns_combo_changed(int)));
		
		vlayout_global_->addWidget(ns_combo_);
		
		table_widget_global_ = new QTableWidget();
		table_widget_global_->setObjectName("table_widget_global");
		table_widget_global_->setRowCount(7);
		table_widget_global_->setColumnCount(4);
		
		//Set Header Label Texts Here
		table_widget_global_->setHorizontalHeaderLabels(QString("Joint name;Slider Position;Edit Position;Joint State").split(";"));
		table_widget_global_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
		
		QString name_of_joint;
		QTextStream stream_name_of_joint(&name_of_joint);
		
		for (size_t i=0; i<7; i++)
		{
			stream_name_of_joint << "Joint" << i;
			table_widget_global_->setItem(i,0,new QTableWidgetItem(stream_name_of_joint.readAll()));
			table_widget_global_->item(i,0)->setFlags(Qt::ItemIsEnabled );
			table_widget_global_->setItem(i,3,new QTableWidgetItem("0"));
			table_widget_global_->item(i,3)->setFlags(Qt::ItemIsEnabled );
			
			stream_name_of_joint.flush();
		}
		
		slider_j0_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j0_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j0_->setValue( 0 );
		connect( slider_j0_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ0(double)) );
		
		table_widget_global_->setCellWidget(0, 1, slider_j0_);
		
		slider_j1_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j1_->setRange((-120 * M_PI / 180), (120 * M_PI / 180), 0.1, 1);
		slider_j1_->setValue( 0 );
		connect( slider_j1_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ1(double)) );
		
		table_widget_global_->setCellWidget(1, 1, slider_j1_);
		
		slider_j2_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j2_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j2_->setValue( 0 );
		connect( slider_j2_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ2(double)) );
		
		table_widget_global_->setCellWidget(2, 1, slider_j2_);
		
		slider_j3_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j3_->setRange((-120 * M_PI / 180), (120 * M_PI / 180), 0.1, 1);
		slider_j3_->setValue( 0 );
		connect( slider_j3_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ3(double)) );
		
		table_widget_global_->setCellWidget(3, 1, slider_j3_);
		
		slider_j4_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j4_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j4_->setValue( 0 );
		connect( slider_j4_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ4(double)) );
		
		table_widget_global_->setCellWidget(4, 1, slider_j4_);
		
		slider_j5_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j5_->setRange((-120 * M_PI / 180), (120 * M_PI / 180), 0.1, 1);
		slider_j5_->setValue( 0 );
		connect( slider_j5_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ5(double)) );
		
		table_widget_global_->setCellWidget(5, 1, slider_j5_);
		
		slider_j6_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
        slider_j6_->setRange((-170 * M_PI / 180), (170 * M_PI / 180), 0.1, 1);
		slider_j6_->setValue( 0 );
		connect( slider_j6_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ6(double)) );
		
		table_widget_global_->setCellWidget(6, 1, slider_j6_);
		
		
		line_j0_ = new QLineEdit();
		line_j0_->setObjectName("line_j0_");
		line_j0_->setSizePolicy(fixed_policy);
		line_j0_->setText(QString::number(slider_j0_->value() ));
		connect( line_j0_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ0()) );
		
		table_widget_global_->setCellWidget(0, 2, line_j0_);
		
		line_j1_ = new QLineEdit();
		line_j1_->setObjectName("line_j1_");
		line_j1_->setSizePolicy(fixed_policy);
		line_j1_->setText(QString::number(slider_j1_->value() ));
		connect( line_j1_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ1()) );
		
		table_widget_global_->setCellWidget(1, 2, line_j1_);
		
		line_j2_ = new QLineEdit();
		line_j2_->setObjectName("line_j2_");
		line_j2_->setSizePolicy(fixed_policy);
		line_j2_->setText(QString::number(slider_j2_->value() ));
		connect( line_j2_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ2()) );
		
		table_widget_global_->setCellWidget(2, 2, line_j2_);
		
		line_j3_ = new QLineEdit();
		line_j3_->setObjectName("line_j3_");
		line_j3_->setSizePolicy(fixed_policy);
		line_j3_->setText(QString::number(slider_j3_->value() ));
		connect( line_j3_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ3()) );
		
		table_widget_global_->setCellWidget(3, 2, line_j3_);
		
		line_j4_ = new QLineEdit();
		line_j4_->setObjectName("line_j4_");
		line_j4_->setSizePolicy(fixed_policy);
		line_j4_->setText(QString::number(slider_j4_->value() ));
		connect( line_j4_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ4()) );
		
		table_widget_global_->setCellWidget(4, 2, line_j4_);
		
		line_j5_ = new QLineEdit();
		line_j5_->setObjectName("line_j5_");
		line_j5_->setSizePolicy(fixed_policy);
		line_j5_->setText(QString::number(slider_j5_->value() ));
		connect( line_j5_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ5()) );
		
		table_widget_global_->setCellWidget(5, 2, line_j5_);
		
		line_j6_ = new QLineEdit();
		line_j6_->setObjectName("line_j6_");
		line_j6_->setSizePolicy(fixed_policy);
		line_j6_->setText(QString::number(slider_j6_->value() ));
		connect( line_j6_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ6()) );
		
		table_widget_global_->setCellWidget(6, 2, line_j6_);
		//table_widget_global_->resizeColumnsToContents();
		
		table_widget_global_->horizontalHeader()->setResizeMode(QHeaderView::Stretch);
		table_widget_global_->verticalHeader()->setResizeMode(QHeaderView::Stretch);

		
		vlayout_global_->addWidget(table_widget_global_);
		
		/* Start : Plot and Curve specifications */
		
		/*
		datas_curve_j0.resize(50);
		//datas_curve_j0.fill(0);
		times_curve_j0.resize(50);
		//times_curve_j0.fill(0);
		
		
		curve_ = new QwtPlotCurve("Joint 0");
		curve_symbol_ = new QwtSymbol(QwtSymbol::Ellipse, QBrush(Qt::blue), QPen(Qt::black), QSize(4,4));
		curve_->setSymbol(curve_symbol_);
		
		plot_ = new QwtPlot(table_widget_global_);
		plot_->setCanvasBackground(Qt::white);
		plot_->setAxisTitle(QwtPlot::yLeft,"Joint Value (radian)");
        plot_->setAxisTitle(QwtPlot::xBottom,"Time");
		
		plot_legend_ = new QwtLegend();
		plot_->insertLegend(plot_legend_, QwtPlot::BottomLegend);
		
		plot_grid_ = new QwtPlotGrid();
		plot_grid_->setPen(QPen(QColor(196,196,196)));
        plot_grid_->attach(plot_);
        
		curve_->setRawSamples(times_curve_j0.data(),datas_curve_j0.data(),50);
		curve_->attach(plot_);
		
		curve_->setPen( QColor( Qt::green ) );
		//plot_->setAxisScale( QwtPlot::xBottom, 1.0, 500.0 );
		//plot_->setAxisScale( QwtPlot::yLeft, 1.0, 500.0 );
		//plot_->setAxisAutoScale(QwtPlot::yLeft);
		//plot_->setAxisAutoScale(QwtPlot::yRight);
		plot_->setAxisScale(QwtPlot::yLeft, (-170 * M_PI / 180), (170 * M_PI / 180));  
		plot_->setTitle( "Plot title" );

		//plot_->replot();
		plot_->show();
		
		
		vlayout_global_->addWidget(plot_);
		*/
		
		plot_checked_ = new platform_sigma_plugins_ns::QtPlotChecked(table_widget_global_, QString("Movement of the KUKA Joints"), QString("Joint Value (radian)"), QString("Time (sec)"), QPair<double,double>((-170 * M_PI / 180), (170 * M_PI / 180)));
		
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
		
		//connect(this, SIGNAL(updateCurves()), this, SLOT(doUpdateCurves()));
		
		timer_ = new QTimer(this);

		// setup signal and slot
		connect(timer_, SIGNAL(timeout()),
          this, SLOT(doUpdateCurves()));

		// msec
		timer_->start(100);
	}
	
	void JointPositionPlugin::ns_combo_changed(int index)
	{
		resetSlidersPositions();	
	}
	
	void JointPositionPlugin::sendPosition()
	{
		map_selected_joint_values_[ns_combo_->currentText()][0] = slider_j0_->value();
		map_selected_joint_values_[ns_combo_->currentText()][1] = slider_j1_->value();
		map_selected_joint_values_[ns_combo_->currentText()][2] = slider_j2_->value();
		map_selected_joint_values_[ns_combo_->currentText()][3] = slider_j3_->value();
		map_selected_joint_values_[ns_combo_->currentText()][4] = slider_j4_->value();
		map_selected_joint_values_[ns_combo_->currentText()][5] = slider_j5_->value();
		map_selected_joint_values_[ns_combo_->currentText()][6] = slider_j6_->value();
			
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
		if (ns_combo_->currentText() == "kuka_lwr_left")
		{
			slider_j0_->setValue(map_current_joint_state_values_["kuka_lwr_left"][0]);
			slider_j1_->setValue(map_current_joint_state_values_["kuka_lwr_left"][1]);
			slider_j2_->setValue(map_current_joint_state_values_["kuka_lwr_left"][2]);
			slider_j3_->setValue(map_current_joint_state_values_["kuka_lwr_left"][3]);
			slider_j4_->setValue(map_current_joint_state_values_["kuka_lwr_left"][4]);
			slider_j5_->setValue(map_current_joint_state_values_["kuka_lwr_left"][5]);
			slider_j6_->setValue(map_current_joint_state_values_["kuka_lwr_left"][6]);
		}
		else
		{
			slider_j0_->setValue(map_current_joint_state_values_["kuka_lwr_right"][0]);
			slider_j1_->setValue(map_current_joint_state_values_["kuka_lwr_right"][1]);
			slider_j2_->setValue(map_current_joint_state_values_["kuka_lwr_right"][2]);
			slider_j3_->setValue(map_current_joint_state_values_["kuka_lwr_right"][3]);
			slider_j4_->setValue(map_current_joint_state_values_["kuka_lwr_right"][4]);
			slider_j5_->setValue(map_current_joint_state_values_["kuka_lwr_right"][5]);
			slider_j6_->setValue(map_current_joint_state_values_["kuka_lwr_right"][6]);
		}
	}
	
	void JointPositionPlugin::setValueLineJ0(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(0, 2)))->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ0()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(0, 1)))->setValue(line_j0_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ1(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(1, 2)))->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ1()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(1, 1)))->setValue(line_j1_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ2(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(2, 2)))->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ2()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(2, 1)))->setValue(line_j2_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ3(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(3, 2)))->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ3()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(3, 1)))->setValue(line_j3_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ4(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(4, 2)))->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ4()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(4, 1)))->setValue(line_j4_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ5(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(5, 2)))->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ5()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(5, 1)))->setValue(line_j5_->text().toDouble());
	}
	
	void JointPositionPlugin::setValueLineJ6(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(6, 2)))->setText(QString::number( value ));
	}
	
	void JointPositionPlugin::updateValueSliderJ6()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(6, 1)))->setValue(line_j6_->text().toDouble());
	}
	
	void JointPositionPlugin::shutdownPlugin()
	{
		shutdownROSComponents_();
		
		disconnect(this, SIGNAL(updateLabelJs(QVector<double>)), this, SLOT(doUpdateLabelJs(QVector<double>)));
		
		disconnect( line_j0_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ0()) );
		disconnect( line_j1_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ1()) );
		disconnect( line_j2_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ2()) );
		disconnect( line_j3_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ3()) );
		disconnect( line_j4_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ4()) );
		disconnect( line_j5_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ5()) );
		disconnect( line_j6_, SIGNAL(returnPressed()), this, SLOT(updateValueSliderJ6()) );
		
		disconnect( slider_j0_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ0(double)) );
		disconnect( slider_j1_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ1(double)) );
		disconnect( slider_j2_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ2(double)) );
		disconnect( slider_j3_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ3(double)) );
		disconnect( slider_j4_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ4(double)) );
		disconnect( slider_j5_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ5(double)) );
		disconnect( slider_j6_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ6(double)) );
		
		disconnect(button_send_, SIGNAL(pressed()), this, SLOT(sendPosition()));
		disconnect(button_reset_, SIGNAL(pressed()), this, SLOT(resetSlidersPositions()));
		
		for (size_t i=0; i<7; i++)
		{
			table_widget_global_->removeRow(i);
		}
		
		if (slider_j0_)
			delete slider_j0_;
			
		if (slider_j1_)
			delete slider_j1_;
		
		if (slider_j2_)
			delete slider_j2_;
		
		if (slider_j3_)
			delete slider_j3_;
		
		if (slider_j4_)
			delete slider_j4_;
		
		if (slider_j5_)
			delete slider_j5_;
		
		if (slider_j6_)
			delete slider_j6_;
			
		if (line_j0_)
			delete line_j0_;
		
		if (line_j1_)
			delete line_j1_;
			
		if (line_j2_)
			delete line_j2_;
		
		if (line_j3_)
			delete line_j3_;
		
		if (line_j4_)
			delete line_j4_;
		
		if (line_j5_)
			delete line_j5_;
		
		if (line_j6_)
			delete line_j6_;
			
		vlayout_global_->removeWidget(ns_combo_);
		vlayout_global_->removeWidget(table_widget_global_);
		vlayout_global_->removeWidget(button_send_);
		vlayout_global_->removeWidget(button_reset_);
		
		if (table_widget_global_)
			delete table_widget_global_;
			
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
		/*
		plot_->setAxisScale(QwtPlot::xBottom, times_curve_j0[0], times_curve_j0[times_curve_j0.size()-1]);
		
		
		plot_->replot();*/
	}
	
	void JointPositionPlugin::jsCallback_left_(const sensor_msgs::JointState::ConstPtr& msg)
	{
		
		QVector<double> values = QVector<double>::fromStdVector(std::vector<double>(std::begin(msg->position), std::end(msg->position)));
		
		bool zeros = std::all_of(values.begin(), values.end(), [](int i) { return i==0; });
		
		if (!zeros)
		{
			map_current_joint_state_values_["kuka_lwr_left"] = values;
			
			double time = msg->header.stamp.sec + (msg->header.stamp.nsec/1e9);
			
			//if (values[1] < 0.1)
				ROS_INFO("time = %f, values[1]  = %f, msg->position[1] = %f" , time, values[1], msg->position[1]);
			
			
			

			if (firstTime_ == 0)
			{
				 firstTime_ = time;
			}

			double timeDuration = time - firstTime_;
			
			/*
			if (datas_curve_j0.size() > 50)
			{
				datas_curve_j0.remove(0);
				times_curve_j0.remove(0);
			}
			
							
			datas_curve_j0.append(v[0]);
			times_curve_j0.append(timeDuration);
				*/
				
			plot_checked_->updateDataCurves(values, timeDuration);
			
			if (ns_combo_->currentText() == "kuka_lwr_left")
			{
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
		
		/*
		QVector<double> v = QVector<double>::fromStdVector(std::vector<double>(std::begin(msg->position), std::end(msg->position)));
		map_current_joint_state_values_["kuka_lwr_right"] = v;
		
		if (ns_combo_->currentText() == "kuka_lwr_right")
		{
			if (map_sliders_is_init_["kuka_lwr_right"]==false)
			{
				resetSlidersPositions();
				map_sliders_is_init_["kuka_lwr_right"]=true;
			}
			
			emit updateLabelJs(v);
		}
		*/
	}
	
	void JointPositionPlugin::doUpdateLabelJs(QVector<double> positions)
	{
		for (size_t i=0; i<positions.size(); i++)
		{
			table_widget_global_->item(i,3)->setText(QString::number(positions[i],'f',5));
		}
	}
	
	void JointPositionPlugin::setupROSComponents_()
	{
		QString name_of_position_controller = "kuka_group_command_controller_fri";
		
		/* Setup publishers */
		map_pub_joint_position_.insert("kuka_lwr_left",getNodeHandle().advertise<std_msgs::Float64MultiArray>(QString("/kuka_lwr_left/").append(name_of_position_controller).append("/").append("command").toStdString(), 1));
		map_pub_joint_position_.insert("kuka_lwr_right",getNodeHandle().advertise<std_msgs::Float64MultiArray>(QString("/kuka_lwr_right/").append(name_of_position_controller).append("/").append("command").toStdString(), 1));
			
		map_sub_joint_handle_.insert("kuka_lwr_left",getNodeHandle().subscribe(QString("/kuka_lwr_left/").append("joint_states").toStdString(), 1000, &JointPositionPlugin::jsCallback_left_, this));
		map_sub_joint_handle_.insert("kuka_lwr_right",getNodeHandle().subscribe(QString("/kuka_lwr_right/").append("joint_states").toStdString(), 1000, &JointPositionPlugin::jsCallback_right_, this));
		
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


