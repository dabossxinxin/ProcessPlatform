#include "ManualThresholdSegmentation.h"
#include "ui_ManualThresholdSegmentation.h"

MTSDialog::MTSDialog(QWidget *parent) :QDialog(parent)
{
	this->setWindowTitle(tr("�ֶ���ֵ�ָ�"));
	okButton = new QPushButton;
	cancelButton = new QPushButton;
	spinBox = new QSpinBox;
	slider = new QSlider;
	createMTSDialog();

	//ui.setupUi(this);
}

void MTSDialog::createMTSDialog()
{
	int minValue = 0;
	int maxValue = 255;
	int singleStep = 1;
	
	// ����΢���������
	spinBox->setMinimum(minValue);
	spinBox->setMaximum(maxValue);
	spinBox->setSingleStep(singleStep);

	// ���û�����������
	slider->setOrientation(Qt::Horizontal);
	slider->setMinimum(minValue);
	slider->setMaximum(maxValue);
	slider->setSingleStep(singleStep);

	// ����΢�����뻬����֮����ź���ۣ��໥�ı䣩
	connect(spinBox, SIGNAL(valueChanged(int)), slider, SLOT(setValue(int)));
	connect(slider, SIGNAL(valueChanged(int)), spinBox, SLOT(setValue(int)));

	QHBoxLayout *topLayout = new QHBoxLayout();
	topLayout->addWidget(spinBox);
	topLayout->addWidget(slider);
	
	okButton->setText(tr("ok"));
	cancelButton->setText(tr("cancel"));
	QHBoxLayout *bottonLayout = new QHBoxLayout();
	bottonLayout->addWidget(okButton);
	bottonLayout->addWidget(cancelButton);

	QVBoxLayout *mainLayout = new QVBoxLayout(this);
	mainLayout->addLayout(topLayout);
	mainLayout->addLayout(bottonLayout);

	connect(okButton, SIGNAL(clicked()), this, SLOT(ok_button_clicked()));
	connect(cancelButton, SIGNAL(clicked()), this, SLOT(cancel_button_clicked()));
	
}

void MTSDialog::ok_button_clicked()
{
	emit send_threshold_data(spinBox->value());
}

void MTSDialog::cancel_button_clicked()
{
	close();
}