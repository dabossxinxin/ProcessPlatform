#ifndef SHOWWIDGET_H
#define SHOWWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QImage>
#include <QTextEdit>
#include <QVTKWidget.h>

class ShowWidget : public QWidget
{
	Q_OBJECT
public:
	explicit ShowWidget(QWidget *parent = 0);
	QImage img;
	QLabel *imageLabel;
	QLabel *resultLabel;
	QTextEdit *text;
	QVTKWidget *qvtkWidget;
signals:
public slots:
};

#endif