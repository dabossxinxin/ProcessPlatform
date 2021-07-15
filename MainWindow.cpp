#include "MainWindow.h"
#include "windows.h"

MainWindow::MainWindow(QWidget *parent)
	: QMainWindow(parent)
{
	//ui.setupUi(this);
	// ��������
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
	
	// ��ʼ����������
	mMyPointCloud.mCloudPtr.reset(new Cloud3D);
	
	// ��ʼ��vtk widget
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
	item0->setText(QApplication::translate("������", "ʱ��", Q_NULLPTR));
	item1->setText(QApplication::translate("������", "����", Q_NULLPTR));
	item2->setText(QApplication::translate("������", "״̬", Q_NULLPTR));
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

	consoleDock->setWindowTitle(QApplication::translate("������", "����̨", Q_NULLPTR));
	
}

void MainWindow::createActions()
{
	// �򿪶���
	openFileAction = new QAction(QIcon("open.png"), tr("��"), this);
	openFileAction->setShortcut(tr("Ctrl+O"));
	openFileAction->setStatusTip(tr("��һ���ļ�"));
	connect(openFileAction, SIGNAL(triggered()), this, SLOT(showOpenFile()));
	
	
	// �½�����
	newFileAction = new QAction(QIcon("new.png"), tr("�½�"), this);
	newFileAction->setShortcut(tr("Ctrl+N"));
	newFileAction->setStatusTip(tr("�½�һ���ļ�"));
	connect(newFileAction, SIGNAL(triggered()), this, SLOT(showNewFile()));

	// �˳�����
	exitAction = new QAction(tr("�˳�"), this);
	exitAction->setShortcut(tr("Ctrl+Q"));
	exitAction->setStatusTip(tr("�˳�����"));
	connect(exitAction, SIGNAL(triggered()), this, SLOT(close()));

	// ���ƶ���
	copyAction = new QAction(QIcon("copy.png"), tr("����"), this);
	copyAction->setShortcut(tr("Ctrl+C"));
	copyAction->setStatusTip(tr("�����ļ�"));
	connect(copyAction, SIGNAL(triggered()), showWidget->text, SLOT(copy()));

	// ���ж�
	cutAction = new QAction(QIcon("cut.png"), tr("����"), this);
	cutAction->setShortcut(tr("Ctrl+X"));
	cutAction->setStatusTip(tr("�����ļ�"));
	connect(cutAction, SIGNAL(triggered()), showWidget->text, SLOT(cut()));
	
	// ճ������
	pasteAction = new QAction(QIcon("paste.png"), tr("ճ��"), this);
	pasteAction->setShortcut(tr("Ctrl+V"));
	pasteAction->setStatusTip(tr("ճ���ļ�"));
	connect(pasteAction, SIGNAL(triggered()), showWidget->text, SLOT(paste()));

	// ���ڶ���
	/*aboutAction = new QAction(tr("����"), this);
	connect(aboutAction, SIGNAL(triggered()), this, SLOT(QApplication::aboutQt()));*/

	// ��ӡ�ı�����
	printTextAction = new QAction(QIcon("printText.png"), tr("��ӡ�ı�"), this);
	printTextAction->setStatusTip("��ӡһ֡�ı�");
	connect(printTextAction, SIGNAL(triggered()), this, SLOT(showPrintText()));

	// ��ӡͼ����
	printImageAction = new QAction(QIcon("printImage.png"), tr("��ӡͼ��"), this);
	printImageAction->setStatusTip(tr("��ӡһ��ͼ��"));
	connect(printImageAction, SIGNAL(triggered()), this, SLOT(showPrintImage()));

	// �Ŵ���
	zoomInAction = new QAction(QIcon("zoomin.png"), tr("�Ŵ�"), this);
	zoomInAction->setStatusTip(tr("�Ŵ�һ��ͼƬ"));
	connect(zoomInAction, SIGNAL(triggered()), this, SLOT(showZoomIn()));

	// ��С����
	zoomOutAction = new QAction(QIcon("zoomOut.png"), tr("��С"), this);
	zoomOutAction->setStatusTip(tr("��Сһ��ͼƬ"));
	connect(zoomOutAction, SIGNAL(triggered()), this, SLOT(showZoomOut()));

	// ʵ��ͼ����ת�Ķ���
	rotate90Action = new QAction(QIcon("rotate90.png"), tr("��ת90"), this);
	rotate90Action->setStatusTip(tr("��һ��ͼ����ת90��"));
	connect(rotate90Action, SIGNAL(triggered()), this, SLOT(showRotate90()));
	rotate180Action = new QAction(QIcon("rotate180.png"), tr("��ת180"), this);
	rotate180Action->setStatusTip(tr("��һ��ͼ����ת180��"));
	connect(rotate180Action, SIGNAL(triggered()), this, SLOT(showRotate180()));
	rotate270Action = new QAction(QIcon("rotate270.png"), tr("��ת270"), this);
	rotate270Action->setStatusTip(tr("��һ��ͼ����ת270��"));
	connect(rotate270Action, SIGNAL(triggered()), this, SLOT(showRotate270()));
	
	// ʵ��ͼ��ľ���Ķ���
	mirrorVerticalAction = new QAction(QIcon("mirrorVertical.png"), tr("������"), this);
	mirrorVerticalAction->setStatusTip(tr("��һ��ͼ����������"));
	connect(mirrorVerticalAction, SIGNAL(triggered()), this, SLOT(showMirrorVertical()));
	mirrorHorizontalAction = new QAction(QIcon("mirrorHorizontal.png"), tr("������"), this);
	mirrorHorizontalAction->setStatusTip(tr("��һ��ͼ����������"));
	connect(mirrorHorizontalAction, SIGNAL(triggered()), this, SLOT(showMirrorHorizontal()));

	// ʵ�ֳ����ͻظ�����
	undoAction = new QAction(QIcon("undo.png"), tr("����"), this);   
	connect(undoAction,SIGNAL(triggered()), showWidget->text, SLOT(undo()));
	redoAction = new QAction(QIcon("redo.png"), tr("����"), this);
	connect(redoAction, SIGNAL(triggered()), showWidget->text, SLOT(redo()));

	// ʵ��ͼ������ֵ�ָ�
	otsuAction = new QAction(QIcon("otsu.png"), tr("OTSU"), this);
	otsuAction->setStatusTip(tr("ʹ�ô�򷨶ԻҶ�ͼ�������ֵ�ָ�"));
	connect(otsuAction, SIGNAL(triggered()), this, SLOT(showOtsuResult()));
	// ʵ���ֶ���ֵ�ָ�
	mtsAction = new QAction(QIcon("mts.png"), tr("MTS"), this);
	mtsAction->setStatusTip(tr("�ֶ�������ֵ��ͼ����зָ�"));
	connect(mtsAction, SIGNAL(triggered()), this, SLOT(showMtsResult()));

	// ����̨����
	consoleAction = new QAction(this);
	consoleAction->setObjectName(QStringLiteral("consoleAction"));
	consoleAction->setCheckable(true);
	consoleAction->setChecked(true);
}

void MainWindow::createMenus()
{
	// �ļ��˵�
	fileMenu = menuBar()->addMenu(tr("�ļ�"));
	fileMenu->addAction(openFileAction);
	fileMenu->addAction(newFileAction);
	fileMenu->addAction(printImageAction);
	fileMenu->addAction(printTextAction);
	fileMenu->addSeparator();
	fileMenu->addAction(exitAction);

	// ���Ų˵�
	zoomMenu = menuBar()->addMenu(tr("�༭"));
	zoomMenu->addAction(copyAction);
	zoomMenu->addAction(cutAction);
	zoomMenu->addAction(pasteAction);
	//zoomMenu->addAction(aboutAction);
	zoomMenu->addSeparator();
	zoomMenu->addAction(zoomInAction);
	zoomMenu->addAction(zoomOutAction);

	// ��ת�˵�
	rotateMenu = menuBar()->addMenu(tr("��ת"));
	rotateMenu->addAction(rotate90Action);
	rotateMenu->addAction(rotate180Action);
	rotateMenu->addAction(rotate270Action);

	// ����˵�
	mirrorMenu = menuBar()->addMenu(tr("����"));
	mirrorMenu->addAction(mirrorVerticalAction);
	mirrorMenu->addAction(mirrorHorizontalAction);
	
	// ���߲˵�
	toolMenu = menuBar()->addMenu("����");
	toolMenu->addAction(otsuAction);
	toolMenu->addAction(mtsAction);
}

void MainWindow::createToolBars()
{
	// �ļ�������
	fileTool = addToolBar("File");
	fileTool->setAllowedAreas(Qt::TopToolBarArea | Qt::LeftToolBarArea);
	fileTool->setMovable(true);		// ָ���ļ������������ƶ�
	fileTool->addAction(openFileAction);
	fileTool->addAction(newFileAction);
	fileTool->addAction(printTextAction);
	fileTool->addAction(printImageAction);

	// �༭������
	zoomTool = addToolBar("Edit");
	zoomTool->setAllowedAreas(Qt::TopToolBarArea | Qt::LeftToolBarArea);
	zoomTool->setMovable(true);		// ָ���ļ������������ƶ�
	zoomTool->addAction(copyAction);
	zoomTool->addAction(cutAction);
	zoomTool->addAction(pasteAction);
	zoomTool->addSeparator();
	zoomTool->addAction(zoomInAction);
	zoomTool->addAction(zoomOutAction);

	// ��ת������
	rotateTool = addToolBar("rotate");
	rotateTool->setAllowedAreas(Qt::TopToolBarArea | Qt::LeftToolBarArea);
	rotateTool->setMovable(true);	// ָ���ļ������������ƶ�
	rotateTool->addAction(rotate90Action);
	rotateTool->addAction(rotate180Action);
	rotateTool->addAction(rotate270Action);

	// ����������������
	doToolBar = addToolBar("doEidt");
	doToolBar->setAllowedAreas(Qt::TopToolBarArea | Qt::LeftToolBarArea);
	doToolBar->setMovable(true);	// ָ���ļ������������ƶ�
	doToolBar->addAction(undoAction);
	doToolBar->addAction(redoAction);
	
	// ����ͼ������������
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
	save_log("�����ļ�" + filename, "�ɹ�");
}

void MainWindow::loadImageFile(const QString& filename)
{
	printf("file name:%s\n", filename.data());
	QImage image;
	image.load(filename);
	img = image;
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(image));
	save_log("�����ļ�" + filename, "�ɹ�");
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
	save_log("�����ļ�" + filename, "�ɹ�");
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
		// �����Ҫ��ӡ��QTextEdit���ĵ�
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
		
		// ����ͼ�εı�����С����������ͼ��������
		size.scale(rect.size(), Qt::KeepAspectRatio);
		painter.setViewport(rect.x(), rect.y(), size.width(), size.height());
		painter.setWindow(img.rect());	// ����QPainter���ڴ�СΪͼ��Ĵ�С
		painter.drawImage(0, 0, img);	// ��ӡͼ��
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
	save_log("�Ŵ�x2", "�ɹ�");
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
	save_log("��Сx2", "�ɹ�");
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
	save_log("��ת90��", "�ɹ�");
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
	save_log("��ת180��", "�ɹ�");
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
	save_log("��ת270��", "�ɹ�");
}

void MainWindow::showMirrorVertical()
{
	if (img.isNull())
		return;

	img = img.mirrored(false, true);
	showWidget->imageLabel->setScaledContents(false);
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(img));
	save_log("��ֱ����ͼƬ", "�ɹ�");
}

void MainWindow::showMirrorHorizontal()
{
	if (img.isNull())
		return;

	img = img.mirrored(true, false);
	showWidget->imageLabel->setScaledContents(false);
	showWidget->imageLabel->setPixmap(QPixmap::fromImage(img));
	save_log("ˮƽ����ͼƬ", "�ɹ�");
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
	save_log("��ֵ�ָ�-OTSU", "�ɹ�");
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
	save_log("��ֵ�ָ�-MTS", "�ɹ�");
}

void MainWindow::showMtsResult()
{
	MTSDialog *mtsDialog = new MTSDialog(this);

	// ����mts�Ի����з�����ֵ���ݵ��źţ����������ֵ��ͼ����д���
	connect(mtsDialog, SIGNAL(send_threshold_data(int)), this, SLOT(receive_threshold_data(int)));

	if (mtsDialog->isHidden())
		mtsDialog->show();
	else mtsDialog->hide();
}