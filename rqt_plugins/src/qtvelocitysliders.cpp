/*
 *  Laurent LEQUIEVRE
 *  CNRS engineer
 *  Institut Pascal UMR6602
 *  laurent.lequievre@uca.fr
 * 
*/

#include "rqt_plugins/qtvelocitysliders.h"

// Qt core
#include <QTextStream>
#include <QMetaType>

// Qt gui
#include <QHeaderView>

namespace platform_sigma_plugins_ns {
	
	QtVelocitySliders::QtVelocitySliders( QWidget *parent) : QWidget(parent), slider_j0_(0), slider_j1_(0), slider_j2_(0), slider_j3_(0), slider_j4_(0), slider_j5_(0), slider_j6_(0),
	line_j0_(0), line_j1_(0), line_j2_(0), line_j3_(0), line_j4_(0), line_j5_(0), line_j6_(0), table_widget_global_(0), vlayout_global_(0)
	{
		createWidget_();
	}
	
	QtVelocitySliders::~QtVelocitySliders()
	{
		disconnect( slider_j0_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ0(double)) );
		disconnect( slider_j1_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ1(double)) );
		disconnect( slider_j2_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ2(double)) );
		disconnect( slider_j3_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ3(double)) );
		disconnect( slider_j4_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ4(double)) );
		disconnect( slider_j5_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ5(double)) );
		disconnect( slider_j6_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ6(double)) );
		
		disconnect( line_j0_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ0()) );
		disconnect( line_j1_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ1()) );
		disconnect( line_j2_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ2()) );
		disconnect( line_j3_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ3()) );
		disconnect( line_j4_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ4()) );
		disconnect( line_j5_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ5()) );
		disconnect( line_j6_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ6()) );
		
		// Deselects all selected items
		table_widget_global_->clearSelection();

		// Disconnect all signals from table widget ! important !
		table_widget_global_->disconnect();

		// Remove all items
		table_widget_global_->clearContents();

		// Set row count to 0 (remove rows)
		table_widget_global_->setRowCount(0);
		
		vlayout_global_->removeWidget(table_widget_global_);
		
		delete table_widget_global_;
		
		delete vlayout_global_;
		
	}
	
	void QtVelocitySliders::createWidget_()
	{
		QSizePolicy fixed_policy(QSizePolicy::Fixed, QSizePolicy::Fixed);
	
		vlayout_global_ = new QVBoxLayout();
		vlayout_global_->setObjectName("vertical_layout_global");
		
		table_widget_global_ = new QTableWidget();
		table_widget_global_->setObjectName("table_widget_global");
		table_widget_global_->setRowCount(7);
		table_widget_global_->setColumnCount(3);
		table_widget_global_->horizontalHeader()->setStretchLastSection(true);
		table_widget_global_->verticalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
		
		//Set Header Label Texts Here
		table_widget_global_->setHorizontalHeaderLabels(QString("Joint name;Edit Velocity;Max Velocity").split(";"));
		
		QString name_of_joint;
		QTextStream stream_name_of_joint(&name_of_joint);
		
		for (size_t i=0; i<7; i++)
		{
			stream_name_of_joint << "Joint" << i;
			table_widget_global_->setItem(i,0,new QTableWidgetItem(stream_name_of_joint.readAll()));
			table_widget_global_->item(i,0)->setFlags(Qt::ItemIsEnabled );
			
			stream_name_of_joint.flush();
		}
		
		slider_j0_ = new QwtSlider(Qt::Horizontal, this);
		slider_j0_->setScale((0.01), (110 * M_PI / 180));
		slider_j0_->setScaleStepSize(0.1);
		slider_j0_->setValue( 0.01 );
		connect( slider_j0_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ0(double)) );
		
		table_widget_global_->setCellWidget(0, 2, slider_j0_);
		
		slider_j1_ = new QwtSlider(Qt::Horizontal, this);
		slider_j1_->setScale((0.01), (110 * M_PI / 180));
		slider_j1_->setScaleStepSize(0.1);
		slider_j1_->setValue( 0.01 );
		connect( slider_j1_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ1(double)) );
		
		table_widget_global_->setCellWidget(1, 2, slider_j1_);
		
		slider_j2_ = new QwtSlider(Qt::Horizontal, this);
		slider_j2_->setScale((0.01), (128 * M_PI / 180));
		slider_j2_->setScaleStepSize(0.1);
		slider_j2_->setValue( 0.01 );
		connect( slider_j2_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ2(double)) );
		
		table_widget_global_->setCellWidget(2, 2, slider_j2_);
		
		slider_j3_ = new QwtSlider(Qt::Horizontal, this);
		slider_j3_->setScale((0.01), (128 * M_PI / 180));
		slider_j3_->setScaleStepSize(0.1);
		slider_j3_->setValue( 0.01 );
		connect( slider_j3_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ3(double)) );
		
		table_widget_global_->setCellWidget(3, 2, slider_j3_);
		
		slider_j4_ = new QwtSlider(Qt::Horizontal, this);
		slider_j4_->setScale((0.01), (204 * M_PI / 180));
		slider_j4_->setScaleStepSize(0.1);
		slider_j4_->setValue( 0.01 );
		connect( slider_j4_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ4(double)) );
		
		table_widget_global_->setCellWidget(4, 2, slider_j4_);
		
		slider_j5_ = new QwtSlider(Qt::Horizontal, this);
		slider_j5_->setScale((0.01), (184 * M_PI / 180));
		slider_j5_->setScaleStepSize(0.1);
		slider_j5_->setValue( 0.01 );
		connect( slider_j5_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ5(double)) );
		
		table_widget_global_->setCellWidget(5, 2, slider_j5_);
		
		slider_j6_ = new QwtSlider(Qt::Horizontal, this);
		slider_j6_->setScale((0.01), (184 * M_PI / 180));
		slider_j6_->setScaleStepSize(0.1);
		slider_j6_->setValue( 0.01 );
		connect( slider_j6_, SIGNAL(valueChanged(double)), this, SLOT(setValueLineJ6(double)) );
		
		table_widget_global_->setCellWidget(6, 2, slider_j6_);
		
		line_j0_ = new QLineEdit();
		line_j0_->setObjectName("line_j0_");
		line_j0_->setSizePolicy(fixed_policy);
		line_j0_->setText(QString::number(slider_j0_->value() ));
		connect( line_j0_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ0()) );
		
		table_widget_global_->setCellWidget(0, 1, line_j0_);
		
		line_j1_ = new QLineEdit();
		line_j1_->setObjectName("line_j1_");
		line_j1_->setSizePolicy(fixed_policy);
		line_j1_->setText(QString::number(slider_j1_->value() ));
		connect( line_j1_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ1()) );
		
		table_widget_global_->setCellWidget(1, 1, line_j1_);
		
		line_j2_ = new QLineEdit();
		line_j2_->setObjectName("line_j2_");
		line_j2_->setSizePolicy(fixed_policy);
		line_j2_->setText(QString::number(slider_j2_->value() ));
		connect( line_j2_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ2()) );
		
		table_widget_global_->setCellWidget(2, 1, line_j2_);
		
		line_j3_ = new QLineEdit();
		line_j3_->setObjectName("line_j3_");
		line_j3_->setSizePolicy(fixed_policy);
		line_j3_->setText(QString::number(slider_j3_->value() ));
		connect( line_j3_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ3()) );
		
		table_widget_global_->setCellWidget(3, 1, line_j3_);
		
		line_j4_ = new QLineEdit();
		line_j4_->setObjectName("line_j4_");
		line_j4_->setSizePolicy(fixed_policy);
		line_j4_->setText(QString::number(slider_j4_->value() ));
		connect( line_j4_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ4()) );
		
		table_widget_global_->setCellWidget(4, 1, line_j4_);
		
		line_j5_ = new QLineEdit();
		line_j5_->setObjectName("line_j5_");
		line_j5_->setSizePolicy(fixed_policy);
		line_j5_->setText(QString::number(slider_j5_->value() ));
		connect( line_j5_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ5()) );
		
		table_widget_global_->setCellWidget(5, 1, line_j5_);
		
		line_j6_ = new QLineEdit();
		line_j6_->setObjectName("line_j6_");
		line_j6_->setSizePolicy(fixed_policy);
		line_j6_->setText(QString::number(slider_j6_->value() ));
		connect( line_j6_, SIGNAL(editingFinished()), this, SLOT(updateValueSliderJ6()) );
		
		table_widget_global_->setCellWidget(6, 1, line_j6_);
		
		vlayout_global_->addWidget(table_widget_global_);
		
		setLayout(vlayout_global_);
	}
	
	void QtVelocitySliders::setValueLineJ0(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(0, 1)))->setText(QString::number( value ));
	}
	
	void QtVelocitySliders::updateValueSliderJ0()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(0, 2)))->setValue(line_j0_->text().toDouble());
	}
	
	void QtVelocitySliders::setValueLineJ1(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(1, 1)))->setText(QString::number( value ));
	}
	
	void QtVelocitySliders::updateValueSliderJ1()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(1, 2)))->setValue(line_j1_->text().toDouble());
	}
	
	void QtVelocitySliders::setValueLineJ2(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(2, 1)))->setText(QString::number( value ));
	}
	
	void QtVelocitySliders::updateValueSliderJ2()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(2, 2)))->setValue(line_j2_->text().toDouble());
	}
	
	void QtVelocitySliders::setValueLineJ3(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(3, 1)))->setText(QString::number( value ));
	}
	
	void QtVelocitySliders::updateValueSliderJ3()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(3, 2)))->setValue(line_j3_->text().toDouble());
	}
	
	void QtVelocitySliders::setValueLineJ4(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(4, 1)))->setText(QString::number( value ));
	}
	
	void QtVelocitySliders::updateValueSliderJ4()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(4, 2)))->setValue(line_j4_->text().toDouble());
	}
	
	void QtVelocitySliders::setValueLineJ5(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(5, 1)))->setText(QString::number( value ));
	}
	
	void QtVelocitySliders::updateValueSliderJ5()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(5, 2)))->setValue(line_j5_->text().toDouble());
	}
	
	void QtVelocitySliders::setValueLineJ6(double value)
	{
		((QLineEdit*)(table_widget_global_->cellWidget(6, 1)))->setText(QString::number( value ));
	}
	
	void QtVelocitySliders::updateValueSliderJ6()
	{
		((QwtSlider*)(table_widget_global_->cellWidget(6, 2)))->setValue(line_j6_->text().toDouble());
	}
	
	void QtVelocitySliders::updateSliders(const QVector<double> & velocities)
	{
		slider_j0_->setValue(velocities[0]);
		slider_j1_->setValue(velocities[1]);
		slider_j2_->setValue(velocities[2]);
		slider_j3_->setValue(velocities[3]);
		slider_j4_->setValue(velocities[4]);
		slider_j5_->setValue(velocities[5]);
		slider_j6_->setValue(velocities[6]);
	}
}
