#ifndef FINDCIRCLESAPPLICATION_H
#define FINDCIRCLESAPPLICATION_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include <iostream>

#pragma execution_character_set("utf-8")

// Qt���ͷ�ļ�
#include <QMessageBox>
#include <QDialog>
#include <QDateTime>
#include <QFileDialog>
#include <QString>
#include <QPixmap>
#include <QPrintDialog>
#include <QPrinter>
#include <QMenu>
#include <QMenuBar>
#include <QAction>
#include <QSpinBox>
#include <QToolBar>
#include <QToolButton>
#include <QTextCharFormat>
#include <QTextStream>
#include <QFontComboBox>
#include <QPainter>
#include <QVTKWidget.h>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QTableWidget>
#include "ShowWidget.h"
#include "Otsu.h"
#include "ManualThresholdSegmentation.h"

// ͼ�������ͷ�ļ�
#include "GBK.h"
#include <sstream>
#include "findCircles.h"
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgcodecs/legacy/constants_c.h>

// ���ƴ������ͷ�ļ�
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include "PointCloud/MyPointCloud.h"

typedef pcl::visualization::PCLVisualizer Viewer;
typedef boost::shared_ptr<pcl::visualization::PCLVisualizer> ViewerPtr;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(QWidget *parent = 0);
	~MainWindow();
	
	void createConsole();
	void createActions();
	void createToolBars();
	void createMenus();
	void loadDocumentFile(const QString& filename);
	void loadImageFile(const QString& filename);
	void loadPointCloudFile(const QString& filename);
	
	void mergeFormat(QTextCharFormat);

private:
	MyPointCloud mMyPointCloud;
	std::vector<MyPointCloud> mCloudVector;
	ViewerPtr mpCloudViewer;

	void initialize();
	void updateViewer(ViewerPtr viewer);

private:
	Ui::findCirclesApplicationClass ui;

	// ����˵���
	QMenu *fileMenu;		
	QMenu *zoomMenu;
	QMenu *rotateMenu;
	QMenu *mirrorMenu;
	QMenu *toolMenu;
	QImage img;
	QString fileName;
	ShowWidget *showWidget;
	// �ļ��˵���
	QAction *openFileAction;
	QAction *newFileAction;
	QAction *printTextAction;
	QAction *printImageAction;
	QAction *exitAction;
	// �༭�˵��� 
	QAction *copyAction;
	QAction *cutAction;
	QAction *pasteAction;
	QAction *aboutAction;
	QAction *zoomInAction;
	QAction *zoomOutAction;
	// ��ת�˵���
	QAction *rotate90Action;
	QAction *rotate180Action;
	QAction *rotate270Action;
	QAction *mirrorVerticalAction;
	QAction *mirrorHorizontalAction;
	QAction *undoAction;
	QAction *redoAction;
	// ͼ����ֵ�ָ�ѡ��
	QAction *otsuAction;
	QAction *mtsAction;
	// ������
	QToolBar *fileTool;
	QToolBar *zoomTool;
	QToolBar *mirrorTool;
	QToolBar *rotateTool;
	QToolBar *doToolBar;
	
	// ����ͼ������������
	QToolBar *imageProcessTool;

	// ����̨����
	QAction *consoleAction;
	QDockWidget *consoleDock;
	QTableWidget *consoleTable;
	

private slots:
	// ����mts�Ի��򴫹�������ֵ����
	void receive_threshold_data(int th);
	
	// ����̨����ļ�
	void save_log(QString operation, QString note);

protected slots:
	void showNewFile();
	void showOpenFile();
	void showPrintText();
	void showPrintImage();
	void showZoomIn();
	void showZoomOut();
	void showRotate90();
	void showRotate180();
	void showRotate270();			
	void showMirrorVertical();		// ��ֱ����
	void showMirrorHorizontal();	// ˮƽ����
	void showOtsuResult();			// ����Զ���ֵ�ָ�
	void showMtsResult();			// �ֶ���ֵ�ָ�

public:
	
};

#endif // FINDCIRCLESAPPLICATION_H
