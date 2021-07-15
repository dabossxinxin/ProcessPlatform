#ifndef FINDCIRCLESAPPLICATION_H
#define FINDCIRCLESAPPLICATION_H

#include <QtWidgets/QMainWindow>
#include "ui_MainWindow.h"
#include <iostream>

#pragma execution_character_set("utf-8")

// Qt相关头文件
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

// 图像处理相关头文件
#include "GBK.h"
#include <sstream>
#include "findCircles.h"
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgcodecs/legacy/constants_c.h>

// 点云处理相关头文件
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

	// 各项菜单栏
	QMenu *fileMenu;		
	QMenu *zoomMenu;
	QMenu *rotateMenu;
	QMenu *mirrorMenu;
	QMenu *toolMenu;
	QImage img;
	QString fileName;
	ShowWidget *showWidget;
	// 文件菜单项
	QAction *openFileAction;
	QAction *newFileAction;
	QAction *printTextAction;
	QAction *printImageAction;
	QAction *exitAction;
	// 编辑菜单项 
	QAction *copyAction;
	QAction *cutAction;
	QAction *pasteAction;
	QAction *aboutAction;
	QAction *zoomInAction;
	QAction *zoomOutAction;
	// 旋转菜单项
	QAction *rotate90Action;
	QAction *rotate180Action;
	QAction *rotate270Action;
	QAction *mirrorVerticalAction;
	QAction *mirrorHorizontalAction;
	QAction *undoAction;
	QAction *redoAction;
	// 图像阈值分割选项
	QAction *otsuAction;
	QAction *mtsAction;
	// 工具栏
	QToolBar *fileTool;
	QToolBar *zoomTool;
	QToolBar *mirrorTool;
	QToolBar *rotateTool;
	QToolBar *doToolBar;
	
	// 基本图像处理方法工具条
	QToolBar *imageProcessTool;

	// 控制台程序
	QAction *consoleAction;
	QDockWidget *consoleDock;
	QTableWidget *consoleTable;
	

private slots:
	// 接收mts对话框传过来的阈值数据
	void receive_threshold_data(int th);
	
	// 控制台输出文件
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
	void showMirrorVertical();		// 垂直镜像
	void showMirrorHorizontal();	// 水平镜像
	void showOtsuResult();			// 大津法自动阈值分割
	void showMtsResult();			// 手动阈值分割

public:
	
};

#endif // FINDCIRCLESAPPLICATION_H
