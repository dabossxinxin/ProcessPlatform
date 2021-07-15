#include "MainWindow.h"
#include <QtWidgets/QApplication>
#include <QPixmap>
#include <QSplashScreen>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	QImage image;
	image.load("splashImage.png");
	QPixmap pixmap = QPixmap::fromImage(image);
	QSplashScreen splash(pixmap);
	splash.show();
	a.processEvents();
	
	MainWindow w;
	w.show();
	splash.finish(&w);
	return a.exec();
}
