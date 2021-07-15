#include "MainWindow.h"
#include "windows.h"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	//ui.setupUi(this);
	// 窗口名称
	setWindowTitle(tr("Image Process Platform"));
	showWidget = new ShowWidget(this);
	setCentralWidget(showWidget);
	createActions();
	createMenus();
	createToolBars();
	createConsole();
	initialize();

	Sleep(500);
}

void MainWindow::initialize()
{
	this->showWidget->qvtkWidget->setVisible(true);
	this->showWidget->imageLabel->setVisible(false);
	this->showWidget->resultLabel->setVisible(false);
	this->showWidget->text->setVisible(false);
	
	// 初始化点云容器
	mMyPointCloud.mCloudPtr.reset(new Cloud3D);
	
	// 初始化vtk widget
	mpCloudViewer.reset(new pcl::visualization::PCLVisualizer("CloudViewer", false));
	this->showWidget->qvtkWidget->SetRenderWindow(mpCloudViewer->getRenderWindow());
	mpCloudViewer->setupInteractor(this->showWidget->qvtkWidget->GetInteractor(),
												this->showWidget->qvtkWidget->GetRenderWindow());
	this->showWidget->qvtkWidget->update();
}

MainWindow::~MainWindow()
{

}

void MainWindow::createConsole()
{
	consoleDock = new QDockWidget(this);
	consoleDock->setObjectName(QStringLiteral("consoleDock"));
	consoleDock->setMinimumSize(QSize(200, 140));
	consoleDock->setMaximumSize(QSize(524287, 140));

	QWidget *dockWidget = new QWidget();
	dockWidget->setObjectName(QStringLiteral("dockWidget"));
	QVBoxLayout *vLayout = new QVBoxLayout(dockWidget);
	vLayout->setSpacing(6);
	vLayout->setContentsMargins(7, 7, 7, 7);
	vLayout->setObjectName(QStringLiteral("vLayout"));

	consoleTable = new QTableWidget(dockWidget);
	if (consoleTable->columnCount() < 3)
		consoleTable->setColumnCount(3);
	QTableWidgetItem *item0 = new QTableWidgetItem();
	QTableWidgetItem *item1 = new QTableWidgetItem();
	QTableWidgetItem *item2 = new QTableWidgetItem();
	item0->setTextAlignment(Qt::AlignLeading | Qt::AlignVCenter);
	item1->setTextAlignment(Qt::AlignLeading | Qt::AlignVCenter);
	item2->setTextAlignment(Qt::AlignLeading | Qt::AlignVCenter);
	item0->setText(QApplication::translate("主窗口", "时间", Q_NULLPTR));
	item1->setText(QApplication::translate("主窗口", "操作", Q_NULLPTR));
	item2->setText(QApplication::translate("主窗口", "状态", Q_NULLPTR));
	consoleTable->setHorizontalHeaderItem(0, item0);
	consoleTable->setHorizontalHeaderItem(1, item1);
	consoleTable->setHorizontalHeaderItem(2, item2);

	consoleTable->setObjectName(QStringLiteral("consoleTable"));
	consoleTable->setShowGrid(false);
	consoleTable->setGridStyle(Qt::SolidLine);
	consoleTable->setRowCount(0);
	consoleTable->setColumnCount(3);
	
	consoleTable->horizontalHeader()->setVisible(true);
	consoleTable->horizontalHeader()->setDefaultSectionSize(200);
	consoleTable->horizontalHeader()->setStretchLastSection(true);
	consoleTable->verticalHeader()->setVisible(false);

	vLayout->addWidget(consoleTable);
	consoleDock->setWidget(dockWidget);
	this->addDockWidget(static_cast<Qt::DockWidgetArea>(8), consoleDock);

	consoleDock->setWindowTitle(QApplication::translate("主窗口", "控制台", Q_NULLPTR));
	
}

void MainWindow::createActions()
{
	// 打开动作
	openFileAction = new QAction(QIcon("open.png"), tr("打开"), this);
	openFileAction->setShortcut(tr("Ctrl+O"));
	openFileAction->setStatusTip(tr("打开一个文件"));
	connect(openFileAction, SIGNAL(triggered()), this, SLOT(showOpenFile()));
	
	
	// 新建动作
	newFileAction = new QAction(QIcon("new.png"), tr("新建"), this);
	newFileAction->setShortcut(tr("Ctrl+N"));
	newFileAction->setStatusTip(tr("新建一个文件"));
	connect(newFileAction, SIGNAL(triggered()), this, SLOT(showNewFile()));

	// 退出动作
	exitAction = new QAction(tr("退出"), this);
	exitAction->setShortcut(tr("Ctrl+Q"));
	exitAction->setStatusTip(tr("退出程序"));
	connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));

	// 复制动作
	copyAction = new QAction(QIcon("copy.png"), tr("复制"), this);
	copyAction->setShortcut(tr("Ctrl+C"));
	copyAction->setStatusTip(tr("复制文件"));
	connect(copyAction, SIGNAL(triggered()), showWidget->text, SLOT(copy()));

	// 剪切动
	cutAction = new QAction(QIcon("cut.png"), tr("剪切"), this);
	cutAction->setShortcut(tr("Ctrl+X"));
	cutAction->setStatusTip(tr("剪切文件"));
	connect(cutAction, SIGNAL(triggered()), showWidget->text, SLOT(cut()));
	
	// 粘贴动作
	pasteAction = new QAction(QIcon("paste.png"), tr("粘贴"), this);
	pasteAction->setShortcut(tr("Ctrl+V"));
	pasteAction->setStatusTip(tr("粘贴文件"));
	connect(pasteAction, SIGNAL(triggered()), showWidget->text, SLOT(paste()));

	// 关于动作
	/*aboutAction = new QAction(tr("关于"), this);
	connect(aboutAction, SIGNAL(triggered()), this, SLOT(QApplication::aboutQt()));*/

	// 打印文本动作
	printTextAction = new QAction(QIcon("printText.png"), tr("打印文本"), this);
	printTextAction->setStatusTip("打印一帧文本");
	connect(printTextAction, SIGNAL(triggered()), this, SLOT(showPrintText()));

	// 打印图像动作
	printImageAction = new QAction(QIcon("printImage.png"), tr("打印图像"), this);
	printImageAction->setStatusTip(tr("打印一幅图像"));
	connect(printImageAction, SIGNAL(triggered()), this, SLOT(showPrintImage()));

	// 放大动作
	zoomInAction = new QAction(QIcon("zoomin.png"), tr("放大"), this);
	zoomInAction->setStatusTip(tr("放大一张图片"));
	connect(zoomInAction, SIGNAL(triggered()), this, SLOT(showZoomIn()));

	// 缩小动作
	zoomOutAction = new QAction(QIcon("zoomOut.png"), tr("缩小"), this);
	zoomOutAction->setStatusTip(tr("缩小一张图片"));
	connect(zoomOutAction, SIGNAL(triggered()), this, SLOT(showZoomOut()));

	// 实现图像旋转的动作
	rotate90Action = new QAction(QIcon("rotate90.png"), tr("旋转90"), this);
	rotate90Action->setStatusTip(tr("将一幅图像旋转90度"));
	connect(rotate90Action, SIGNAL(triggered()), this, SLOT(showRotate90()));
	rotate180Action = new QAction(QIcon("rotate180.png"), tr("旋转180"), this);
	rotate180Action->setStatusTip(tr("将一幅图像旋转180度"));
	connect(rotate180Action, SIGNAL(triggered()), this, SLOT(showRotate180()));
	rotate270Action = new QAction(QIcon("rotate270.png"), tr("旋转270"), this);
	rotate270Action->setStatusTip(tr("将一幅图像旋转270度"));
	connect(rotate270Action, SIGNAL(triggered()), this, SLOT(showRotate270()));
	
	// 实现图像的镜像的动作
	mirrorVerticalAction = new QAction(QIcon("mirrorVertical.png"), tr("纵向镜像"), this);
	mirrorVerticalAction->setStatusTip(tr("对一幅图像做纵向镜像"));
	connect(mirrorVerticalAction, SIGNAL(triggered()), this, SLOT(showMirrorVertical()));
	mirrorHorizontalAction = new QAction(QIcon("mirrorHorizontal.png"), tr("横向镜像"), this);
	mirrorHorizontalAction->setStatusTip(tr("对一幅图像做横向镜像"));
	connect(mirrorHorizontalAction, SIGNAL(triggered()), this, SLOT(showMirrorHorizontal()));

	// 实现撤销和回复动作
	undoAction = new QAction(QIcon("undo.png"), tr("撤销"), this);   
	connect(undoAction,SIGNAL(triggered()), showWidget->text, SLOT(undo()));
	redoAction = new QAction(QIcon("redo.png"), tr("重做"), this);
	connect(redoAction, SIGNAL(triggered()), showWidget->text, SLOT(redo()));

	// 实现图像大津法阈值分割
	otsuAction = new QAction(QIcon("otsu.png"), tr("OTSU"), this);
	otsuAction->setStatusTip(tr("使用大津法对灰度图像进行阈值分割"));
	connect(otsuAction, SIGNAL(triggered()), this, SLOT(showOtsuResult()));
	// 实现手动阈值分割
	mtsAction = new QAction(QIcon("mts.png"), tr("MTS"), this);
	mtsAction->setStatusTip(tr("手动设置阈值对图像进行分割"));
	connect(mtsAction, SIGNAL(triggered()), this, SLOT(showMtsResult()));

	// 控制台程序
	consoleAction = new QAction(this);
	consoleAction->setObjectName(QStringLiteral("consoleAction"));
	consoleAction->setCheckable(true);
	consoleAction->setChecked(true);
}

void MainWindow::createMenus()
{
	// 文件菜单
	fileMenu = menuBar()->addMenu(tr("文件"));
	fileMenu->addAction(openFileAction);
	fileMenu->addAction(newFileAction);
	fileMenu->addAction(printImageAction);
	fileMenu->addAction(printTextAction);
	fileMenu->addSeparator();
	fileMenu->addAction(exitAction);

	// 缩放菜单
	zoomMenu = menuBar()->addMenu(tr("编辑"));
	zoomMenu->addAction(copyAction);
	zoomMenu->addAction(cutAction);
	zoomMenu->addAction(pasteAction);
	//zoomMenu->addAction(aboutAction);
	zoomMenu->addSeparator();
	zoomMenu->addAction(zoomInAction);
	zoomMenu->addAction(zoomOutAction);

	// 旋转菜单
	rotateMenu = menuBar()->addMenu(tr("旋转"));
	rotateMenu->addAction(rotate90Action);
	rotateMenu->addAction(rotate180Action);
	rotateMenu->addAction(rotate270Action);

	// 镜像菜单
	mirrorMenu = menuBar()->addMenu(tr("镜像"));
	mirrorMenu->addAction(mirrorVerticalAction);
	mirrorMenu->addAction(mirrorHorizontalAction);
	
	// 工具菜单
	toolMenu = menuBar()->addMenu("工具");
	toolMenu->addAction(otsuAction);
	toolMenu->addAction(mtsAction);
}

void MainWindow::createToolBars()
{
	// 文件工具条
	fileTool = addToolBar("File");
	fileTool->setAllowedAreas(Qt::TopToolBarArea | Qt::LeftToolBarArea);
	fileTool->setMovable(true);		// 指定文件工具条不可移动
	fileTool->addAction(openFileAction);
	fileTool->addAction(newFileAction);
	fileTool->addAction(printTextAction);
	fileTool->addAction(printImageAction);

	// 编辑工具条
	zoomTool = addToolBar("Edit");
	zoomTool->setAllowedAreas(Qt::TopToolBarArea | Qt::LeftToolBarArea);
	zoomTool->setMovable(true);		// 指定文件工具条不可移动
	zoomTool->addAction(copyAction);
	zoomTool->addAction(cutAction);
	zoomTool->addAction(pasteAction);
	zoomTool->addSeparator();
	zoomTool->addAction(zoomInAction);
	zoomTool->addAction(zoomOutAction);

	// 旋转工具条
	rotateTool = addToolBar("rotate");
	rotateTool->setAllowedAreas(Qt::TopToolBarArea | Qt::LeftToolBarArea);
	rotateTool->setMovable(true);	// 指定文件工具条不可移动
	rotateTool->addAction(rotate90Action);
	rotateTool->addAction(rotate180Action);
	rotateTool->addAction(rotate270Action);

	// 撤销和重做工具条
	doToolBar = addToolBar("doEidt");
	doToolBar->setAllowedAreas(Qt::TopToolBarArea | Qt::LeftToolBarArea);
	doToolBar->setMovable(true);	// 指定文件工具条不可移动
	doToolBar->addAction(undoAction);
	doToolBar->addAction(redoAction);
	
	// 基本图像处理方法工具条
	imageProcessTool = addToolBar("image process");
	imageProcessTool->setAllowedAreas(Qt::TopToolBarArea | Qt::LeftToolBarArea);
	imageProcessTool->setMovable(true);
	imageProcessTool->addAction(otsuAction);
	imageProcessTool->addAction(mtsAction);
}


void MainWindow::showNewFile()
{
	MainWindow *newMainWindow = new MainWindow;
	newMainWindow->show();
}

void MainWindow::loadDocumentFile(const QString& filename)
{
	printf("file name:%s\n", filename.data());
	QFile file(filename);
	if (file.open(QIODevice::ReadOnly | QIODevice::Text))
	{
		QTextStream textStream(&file);
		while (!textStream.atEnd())
		{
			showWidget->text->append(textStream.readLine());
			printf("read line\n");
		}
		printf("end\n");
	}
	save_log("加载文件" + filename, "成功");
}

void MainWindow::loadImageFile(const QString& filename)
{
	printf("file name:%s\n", filename.data());
	QImage image;
	image.load(filename);
	img = image;
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(image));
	save_log("加载文件" + filename, "成功");
}

void MainWindow::loadPointCloudFile(const QString& filename)
{
	printf("file name:%s\n", filename.data());
	QFileInfo fileInfo;
	fileInfo = QFileInfo(filename);
	std::string file_name = filename.toStdString();
	std::string sub_name = fileInfo.suffix().toStdString();
	pcl::io::loadPCDFile(file_name, *(mMyPointCloud.mCloudPtr));

	mMyPointCloud.mFileName = file_name;
	mMyPointCloud.mSuffix = sub_name;
	mCloudVector.push_back(mMyPointCloud);

	mpCloudViewer->removeAllPointClouds();
	updateViewer(mpCloudViewer);
	save_log("加载文件" + filename, "成功");
}

void MainWindow::updateViewer(ViewerPtr viewer)
{
	const int num = mCloudVector.size();
	for (int it = 0; it < num; it++)
	{
		mpCloudViewer->addPointCloud(mCloudVector[it].mCloudPtr, "cloud" + to_string(it));
		mpCloudViewer->updatePointCloud(mCloudVector[it].mCloudPtr, "cloud" + to_string(it));
	}
	mpCloudViewer->resetCamera();
	this->showWidget->qvtkWidget->update();
}

void MainWindow::save_log(QString operation, QString note)
{
	int rows = this->consoleTable->rowCount();
	this->consoleTable->setRowCount(++rows);
	QDateTime now = QDateTime::currentDateTime();
	QString now_str = now.toString("MM-dd hh:mm:ss");
	this->consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(now_str));
	this->consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(operation));
	this->consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(note));
}

void MainWindow::showOpenFile()
{
	QFileInfo fileInfo;
	fileName = QFileDialog::getOpenFileName(this);
	fileInfo = QFileInfo(fileName);

	if (!fileName.isEmpty())
	{
		//std::cout << fileInfo.suffix().toStdString().data() << std::endl;
		if (fileInfo.suffix() == "txt")
		{
			showWidget->text->clear();
			this->loadDocumentFile(fileName);
		}
		
		if (fileInfo.suffix() == "jpg" || fileInfo.suffix() == "png")
		{
			showWidget->imageLabel->clear();
			showWidget->resultLabel->clear();
			this->loadImageFile(fileName);
		}
	}
}

void MainWindow::showPrintText()
{
	QPrinter printer;
	QPrintDialog printDialog(&printer, this);
	if (printDialog.exec())
	{
		// 获得需要打印的QTextEdit的文档
		QTextDocument *doc = showWidget->text->document();
		doc->print(&printer);
	}
}

void MainWindow::showPrintImage()
{
	QPrinter printer;
	QPrintDialog printDialog(&printer, this);
	if (printDialog.exec())
	{
		QPainter painter(&printer);
		QRect rect = painter.viewport();
		QSize size = img.size();
		
		// 按照图形的比例大小重新设置视图矩形区域
		size.scale(rect.size(), Qt::KeepAspectRatio);
		painter.setViewport(rect.x(), rect.y(), size.width(), size.height());
		painter.setWindow(img.rect());	// 设置QPainter窗口大小为图像的大小
		painter.drawImage(0, 0, img);	// 打印图像
	}
}

void MainWindow::showZoomIn()
{
	if (img.isNull())
		return;

	QMatrix matrix;
	matrix.scale(2, 2);
	img = img.transformed(matrix);
	showWidget->imageLabel->setScaledContents(false);
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(img));
	save_log("放大x2", "成功");
}

void MainWindow::showZoomOut()
{
	if (img.isNull())
		return;

	QMatrix matrix;
	matrix.scale(0.5, 0.5);
	img = img.transformed(matrix);
	showWidget->imageLabel->setScaledContents(false);
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(img));
	save_log("缩小x2", "成功");
}

void MainWindow::showRotate90()
{
	if (img.isNull())
		return;

	QMatrix matrix;
	matrix.rotate(90);
	img = img.transformed(matrix);
	showWidget->imageLabel->setScaledContents(false);
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(img));
	save_log("旋转90度", "成功");
}

void MainWindow::showRotate180()
{
	if (img.isNull())
		return;

	QMatrix matrix;
	matrix.rotate(180);
	img = img.transformed(matrix);
	showWidget->imageLabel->setScaledContents(false);
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(img));
	save_log("旋转180度", "成功");
}

void MainWindow::showRotate270()
{
	if (img.isNull())
		return;

	QMatrix matrix;
	matrix.rotate(270);
	img = img.transformed(matrix);
	showWidget->imageLabel->setScaledContents(false);
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(img));
	save_log("旋转270度", "成功");
}

void MainWindow::showMirrorVertical()
{
	if (img.isNull())
		return;

	img = img.mirrored(false, true);
	showWidget->imageLabel->setScaledContents(false);
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(img));
	save_log("竖直镜像图片", "成功");
}

void MainWindow::showMirrorHorizontal()
{
	if (img.isNull())
		return;

	img = img.mirrored(true, false);
	showWidget->imageLabel->setScaledContents(false);
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(img));
	save_log("水平镜像图片", "成功");
}

void MainWindow::showOtsuResult()
{
	if (img.isNull())
		return;
	
	cv::Mat gray;
	cv::Mat mat = Otsu::QImage2cvMat(img);
	if (mat.channels() == 3) cvtColor(mat, gray, CV_BGR2GRAY);
	else if (mat.channels() == 4) cvtColor(mat, gray, CV_RGBA2GRAY);
	else if (mat.channels() == 1) gray = mat;
	else std::cerr << "ERROR:the format of image is error." << std::endl;

	int th = Otsu::threshold(gray);
	cv::Mat result(gray.rows, gray.cols, gray.type());
	
	for (int row = 0; row < gray.rows; row++)
	{
		for (int col = 0; col < gray.cols; col++)
		{
			if (gray.at<uchar>(row, col) > th)
				result.at<uchar>(row, col) = gray.at<uchar>(row, col);
			else
				result.at<uchar>(row, col) = 0;
		}
	}
	QImage image = Otsu::cvMat2QImage(result);
	showWidget->resultLabel->setPixmap(QPixmap::fromImage(image));
	save_log("阈值分割-OTSU", "成功");
}

void MainWindow::receive_threshold_data(int th)
{
	if (img.isNull())
		return;

	cv::Mat gray;
	cv::Mat mat = Otsu::QImage2cvMat(img);
	if (mat.channels() == 3) cvtColor(mat, gray, CV_BGR2GRAY);
	else if (mat.channels() == 4) cvtColor(mat, gray, CV_RGBA2GRAY);
	else if (mat.channels() == 1) gray = mat;
	else std::cerr << "ERROR:the format of image is error." << std::endl;

	cv::Mat result(gray.rows, gray.cols, gray.type());
	for (int row = 0; row < gray.rows; row++)
	{
		for (int col = 0; col < gray.cols; col++)
		{
			if (gray.at<uchar>(row, col) > th)
				result.at<uchar>(row, col) = gray.at<uchar>(row, col);
			else
				result.at<uchar>(row, col) = 0;
		}
	}
	QImage image = Otsu::cvMat2QImage(result);
	showWidget->resultLabel->setPixmap(QPixmap::fromImage(image));
	save_log("阈值分割-MTS", "成功");
}

void MainWindow::showMtsResult()
{
	MTSDialog *mtsDialog = new MTSDialog(this);

	// 触发mts对话框中发送阈值数据的信号，根据这个阈值对图像进行处理
	connect(mtsDialog, SIGNAL(send_threshold_data(int)), this, SLOT(receive_threshold_data(int)));

	if (mtsDialog->isHidden())
		mtsDialog->show();
	else mtsDialog->hide();
}