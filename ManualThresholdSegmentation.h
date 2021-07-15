#ifndef MANUALTHRESHOLDSEGMENTATION_H
#define MANUALTHRESHOLDSEGMENTATION_H

#pragma execution_character_set("utf-8")

#include <QDialog>
#include <QLabel>
#include <QSpinBox>
#include <QSlider>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QDialogButtonBox>
#include "ui_ManualThresholdSegmentation.h"

class MTSDialog : public QDialog
{
	Q_OBJECT
public:
	MTSDialog(QWidget *parent = 0);
	~MTSDialog() {};
signals:
	void send_threshold_data(int);
private slots:
	void ok_button_clicked();
	void cancel_button_clicked();
private:
	Ui::Dialog ui;

	void createMTSDialog();
	QSpinBox *spinBox;
	QSlider *slider;
	QPushButton *okButton;
	QPushButton *cancelButton;
};

#endif