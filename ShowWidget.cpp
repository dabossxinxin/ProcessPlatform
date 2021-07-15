#include "ShowWidget.h"
#include <QHBoxLayout>

ShowWidget::ShowWidget(QWidget *parent) : QWidget(parent)
{
	imageLabel = new QLabel;
	resultLabel = new QLabel;

	qvtkWidget = new QVTKWidget;
	
	imageLabel->setScaledContents(false);
	resultLabel->setScaledContents(false);
	text = new QTextEdit;

	QHBoxLayout *mainLayout = new QHBoxLayout(this);
	mainLayout->setSpacing(6);
	mainLayout->setContentsMargins(7, 7, 7, 7);
	mainLayout->addWidget(qvtkWidget);
	mainLayout->addWidget(imageLabel);
	mainLayout->addWidget(resultLabel);
	mainLayout->addWidget(text);
}