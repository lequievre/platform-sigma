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

namespace platform_sigma_plugins_ns {
	
	JointPositionPlugin::JointPositionPlugin()
	: rqt_gui_cpp::Plugin(), widget_sliders_(0), tab_widget_(0), hlayout_j0_(0), hlayout_j1_(0), hlayout_j2_(0), hlayout_j3_(0), hlayout_j4_(0), hlayout_j5_(0), hlayout_j6_(0), 
	 label_j0_(0), label_j1_(0), label_j2_(0), label_j3_(0), label_j4_(0), label_j5_(0), label_j6_(0), 
	 button_send_(0), button_reset_(0),
	 slider_j0_(0), slider_j1_(0), slider_j2_(0), slider_j3_(0), slider_j4_(0), slider_j5_(0), slider_j6_(0),
	 line_j0_(0), line_j1_(0), line_j2_(0), line_j3_(0), line_j4_(0), line_j5_(0), line_j6_(0), current_time(0), previous_time(0)
	{
		setObjectName("Plugin Joint Position");
		qRegisterMetaType<QVector<double> >("QVector<double>");
	}
	
	void JointPositionPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		
		// create a main widget named widget_
		widget_sliders_ = new QWidget();
		widget_sliders_->setWindowTitle("Joint Position");
	
		// create layouts
		vlayout_outer_ = new QVBoxLayout();
		vlayout_outer_->setObjectName("vertical_layout_outer");
		
		QSizePolicy fixed_policy(QSizePolicy::Fixed, QSizePolicy::Fixed);
		
		// layout ns
		hlayout_ns_= new QHBoxLayout();
		hlayout_ns_->setObjectName("horizontal_layout_ns");
		
		ns_label_ = new QLabel();
        ns_label_->setObjectName("ns_label_");
        ns_label_->setText("Kuka Namespace :");
        ns_label_->setSizePolicy(fixed_policy);
        hlayout_ns_->addWidget(ns_label_);
        
        // create a combo box for kuka namespaces
        ns_combo_ = new QComboBox();
        ns_combo_->setObjectName("ns_combo_");
        ns_combo_->addItem("kuka_lwr_left");
        ns_combo_->addItem("kuka_lwr_right");
		hlayout_ns_->addWidget(ns_combo_);
		
		connect(ns_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(ns_combo_changed(int)));
		
		vlayout_outer_->addLayout(hlayout_ns_);
		
		// start j0
		hlayout_j0_ = new QHBoxLayout();
        hlayout_j0_->setObjectName("horizontal_layout_j0");
        
        label_j0_ = new QLabel();
        label_j0_->setObjectName("label_j0_");
        label_j0_->setText("Joint 0 :");
        label_j0_->setSizePolicy(fixed_policy);
        hlayout_j0_->addWidget(label_j0_);
        
        slider_j0_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
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
		//hlayout_j0_->addStretch(1);
		
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
        
        slider_j1_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
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
        
        slider_j2_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
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
        
        slider_j3_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
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
        
        slider_j4_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
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
        
        slider_j5_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
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
        
        slider_j6_ = new QwtSlider(widget_sliders_, Qt::Horizontal, QwtSlider::TopScale, QwtSlider::Trough );
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
		
		/*connect(this, SIGNAL(updateLabelJs0(double)), this, SLOT(doUpdateLabelJs0(double)));
		connect(this, SIGNAL(updateLabelJs1(double)), this, SLOT(doUpdateLabelJs1(double)));
		connect(this, SIGNAL(updateLabelJs2(double)), this, SLOT(doUpdateLabelJs2(double)));
		connect(this, SIGNAL(updateLabelJs3(double)), this, SLOT(doUpdateLabelJs3(double)));
		connect(this, SIGNAL(updateLabelJs4(double)), this, SLOT(doUpdateLabelJs4(double)));
		connect(this, SIGNAL(updateLabelJs5(double)), this, SLOT(doUpdateLabelJs5(double)));
		connect(this, SIGNAL(updateLabelJs6(double)), this, SLOT(doUpdateLabelJs6(double)));*/
		
		connect(this, SIGNAL(updateLabelJs(QVector<double>)), this, SLOT(doUpdateLabelJs(QVector<double>)));
		
		// set widget_ to main widget
		widget_sliders_->setLayout(vlayout_outer_);
		
		table_widget_state_ = new QTableWidget();
		table_widget_state_->setObjectName("table_widget_state_");
		table_widget_state_->setRowCount(7);
		table_widget_state_->setColumnCount(2);
		
		table_widget_state_->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
	
		//Set Header Label Texts Here
		table_widget_state_->setHorizontalHeaderLabels(QString("Joint name;Joint State").split(";"));
		
		QString string_name_of_joint;
		QTextStream stream_name_of_joint(&string_name_of_joint);
		
		for (size_t i=0; i<7; i++)
		{
			stream_name_of_joint << "Joint" << i;
			table_widget_state_->setItem(i,0,new QTableWidgetItem(stream_name_of_joint.readAll()));
			table_widget_state_->setItem(i,1,new QTableWidgetItem("0"));
			
			stream_name_of_joint.flush();
		}
		
		tab_widget_ = new QTabWidget();
		tab_widget_->addTab(widget_sliders_,"Sliders Position");
		tab_widget_->addTab(table_widget_state_,"Joint State");
		
		context.addWidget(tab_widget_);
		
		QVector<double> vect_init_joint_values;
		vect_init_joint_values.resize(7);
		vect_init_joint_values.fill(0);
		
		map_selected_joint_values_.insert("kuka_lwr_left",vect_init_joint_values);
		map_selected_joint_values_.insert("kuka_lwr_right",vect_init_joint_values);
		
		setupROSComponents_();
	}
	
	void JointPositionPlugin::ns_combo_changed(int index)
	{
		slider_j0_->setValue(map_selected_joint_values_[ns_combo_->currentText()][0]);
		line_j0_->setText(QString::number(map_selected_joint_values_[ns_combo_->currentText()][0]));
		
		slider_j1_->setValue(map_selected_joint_values_[ns_combo_->currentText()][1]);
		line_j1_->setText(QString::number(map_selected_joint_values_[ns_combo_->currentText()][1]));
		
		slider_j2_->setValue(map_selected_joint_values_[ns_combo_->currentText()][2]);
		line_j2_->setText(QString::number(map_selected_joint_values_[ns_combo_->currentText()][2]));
		
		slider_j3_->setValue(map_selected_joint_values_[ns_combo_->currentText()][3]);
		line_j3_->setText(QString::number(map_selected_joint_values_[ns_combo_->currentText()][3]));
		
		slider_j4_->setValue(map_selected_joint_values_[ns_combo_->currentText()][4]);
		line_j4_->setText(QString::number(map_selected_joint_values_[ns_combo_->currentText()][4]));
		
		slider_j5_->setValue(map_selected_joint_values_[ns_combo_->currentText()][5]);
		line_j5_->setText(QString::number(map_selected_joint_values_[ns_combo_->currentText()][5]));
		
		slider_j6_->setValue(map_selected_joint_values_[ns_combo_->currentText()][6]);
		line_j6_->setText(QString::number(map_selected_joint_values_[ns_combo_->currentText()][6]));
		
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
	
	void JointPositionPlugin::resetPosition()
	{
		slider_j0_->setValue(0);
		slider_j1_->setValue(0);
		slider_j2_->setValue(0);
		slider_j3_->setValue(0);
		slider_j4_->setValue(0);
		slider_j5_->setValue(0);
		slider_j6_->setValue(0);
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
	
	void JointPositionPlugin::jsCallback_left_(const sensor_msgs::JointState::ConstPtr& msg)
	{
		if (ns_combo_->currentText() == "kuka_lwr_left")
		{
			/*emit updateLabelJs0(msg->position[0]);
			emit updateLabelJs1(msg->position[1]);
			emit updateLabelJs2(msg->position[2]);
			emit updateLabelJs3(msg->position[3]);
			emit updateLabelJs4(msg->position[4]);
			emit updateLabelJs5(msg->position[5]);
			emit updateLabelJs6(msg->position[6]);*/
			QVector<double> v = QVector<double>::fromStdVector(std::vector<double>(std::begin(msg->position), std::end(msg->position)));
			emit updateLabelJs(v);
			
		}
	}
	
	void JointPositionPlugin::jsCallback_right_(const sensor_msgs::JointState::ConstPtr& msg)
	{
		if (ns_combo_->currentText() == "kuka_lwr_right")
		{
			/*emit updateLabelJs0(msg->position[0]);
			emit updateLabelJs1(msg->position[1]);
			emit updateLabelJs2(msg->position[2]);
			emit updateLabelJs3(msg->position[3]);
			emit updateLabelJs4(msg->position[4]);
			emit updateLabelJs5(msg->position[5]);
			emit updateLabelJs6(msg->position[6]);*/
			//std::vector<double> sv(std::begin(msg->position), std::end(msg->position));
			QVector<double> v = QVector<double>::fromStdVector(std::vector<double>(std::begin(msg->position), std::end(msg->position)));
			emit updateLabelJs(v);
		}
	}
	
	
	void JointPositionPlugin::doUpdateLabelJs(QVector<double> positions)
	{
		for (size_t i=0; i<positions.size(); i++)
		{
			table_widget_state_->item(i,1)->setText(QString::number(positions[i],'f',5));
		}
	}
	
	
	/*void JointPositionPlugin::doUpdateLabelJs0(double position)
	{
		//label_js_j0_->setText(QString::number(position,'f',3));
		table_widget_state_->item(0,1)->setText(QString::number(position,'f',5));
	}
	
	
	void JointPositionPlugin::doUpdateLabelJs1(double position)
	{
		//label_js_j1_->setText(QString::number(position,'f',3));
		table_widget_state_->item(1,1)->setText(QString::number(position,'f',5));
	}
	
	void JointPositionPlugin::doUpdateLabelJs2(double position)
	{
		//label_js_j2_->setText(QString::number(position,'f',3));
		table_widget_state_->item(2,1)->setText(QString::number(position,'f',5));
	}
	
	void JointPositionPlugin::doUpdateLabelJs3(double position)
	{
		//label_js_j3_->setText(QString::number(position,'f',3));
		table_widget_state_->item(3,1)->setText(QString::number(position,'f',5));
	}
	
	void JointPositionPlugin::doUpdateLabelJs4(double position)
	{
		//label_js_j4_->setText(QString::number(position,'f',3));
		table_widget_state_->item(4,1)->setText(QString::number(position,'f',5));
	}
	
	void JointPositionPlugin::doUpdateLabelJs5(double position)
	{
		//label_js_j5_->setText(QString::number(position,'f',3));
		table_widget_state_->item(5,1)->setText(QString::number(position,'f',5));
	}
	
	void JointPositionPlugin::doUpdateLabelJs6(double position)
	{
		//label_js_j6_->setText(QString::number(position,'f',3));
		table_widget_state_->item(6,1)->setText(QString::number(position,'f',5));
	}
	*/
	
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
		
	}
	
} // End of namespace

PLUGINLIB_DECLARE_CLASS(platform_sigma_plugins_ns, JointPositionPlugin, platform_sigma_plugins_ns::JointPositionPlugin, rqt_gui_cpp::Plugin)


