#ifndef OTSU_H
#define OTSU_H

#include <QImage>
#include <QDebug>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>

class Otsu 
{
public:
	static int threshold(const cv::Mat &image)
	{
		int th;
		cv::Mat gray;
		const int grayScale = 256;
		int pixCount[grayScale] = { 0 };
		int pixSum = image.rows*image.cols;
		float pixProportion[grayScale] = { 0. };

		float w0, w1, u0tmp, u1tmp, u0, u1, deltaTmp, deltaMax = 0;

		if (image.channels() == 3) cvtColor(image, gray, CV_BGR2GRAY);
		else if (image.channels() == 4) cvtColor(image, gray, CV_RGBA2GRAY);
		else gray = image;

		// Count the number of pixels in each gray level
		for (int i = 0; i < gray.cols; i++)
		{
			for (int j = 0; j < gray.rows; j++)
			{
				int size = gray.at<uchar>(j, i);
				pixCount[gray.at<uchar>(j, i)]++;
			}
		}

		for (int i = 0; i < grayScale; i++)
			pixProportion[i] = pixCount[i] * 1.0 / pixSum;

		// Traverse the threshold segmentation conditions of all gray levels and test which one has the largest inter-class variance
		for (int i = 0; i < grayScale; i++)
		{
			w0 = w1 = u0tmp = u1tmp = u0 = u1 = deltaTmp = 0;
			for (int j = 0; j < grayScale; j++)
			{
				if (j <= i)		// background
				{
					w0 += pixProportion[j];
					u0tmp += j * pixProportion[j];
				}
				else			// foreground
				{
					w1 += pixProportion[j];
					u1tmp += j * pixProportion[j];
				}
			}
			u0 = u0tmp / w0;
			u1 = u1tmp / w1;

			// Between-class variance formula: g = w1 * w2 * (u1 - u2) ^ 2
			deltaTmp = (float)(w0 * w1 * pow((u0 - u1), 2));
			if (deltaTmp > deltaMax)
			{
				deltaMax = deltaTmp;
				th = i;
			}
		}
		return th;
	}

	static cv::Mat QImage2cvMat(QImage &image)
	{
		cv::Mat mat;
		//std::cout << image.format() << std::endl;
		switch (image.format())
		{
		case QImage::Format_ARGB32:
		case QImage::Format_RGB32:
		case QImage::Format_ARGB32_Premultiplied:
			mat = cv::Mat(image.height(), image.width(), CV_8UC4, (void*)image.constBits(), image.bytesPerLine());
			break;
		case QImage::Format_RGB888:
			mat = cv::Mat(image.height(), image.width(), CV_8UC3, (void*)image.constBits(), image.bytesPerLine());
			break;
		case QImage::Format_Indexed8:
			mat = cv::Mat(image.height(), image.width(), CV_8UC1, (void*)image.constBits(), image.bytesPerLine());
		}
		return mat;
	}

	static QImage cvMat2QImage(const cv::Mat &mat)
	{
		// 8 bits unsigned channel = 1
		if (mat.type() == CV_8UC1)
		{
			QImage image(mat.cols, mat.rows, QImage::Format_Indexed8);
			// set the color table
			image.setColorCount(256);
			for (int i = 0; i < 256; i++)
			{
				image.setColor(i, qRgb(i, i, i));
			}
			// copy input mat
			uchar *pSrc = mat.data;
			for (int row = 0; row < mat.rows; row++)
			{
				uchar *pDest = image.scanLine(row);
				memcpy(pDest, pSrc, mat.cols);
				pSrc += mat.step;
			}
			return image;
		}

		// 8 bits unsigned channel = 3
		else if (mat.type() == CV_8UC3)
		{
			std::cout << "CV-8UC3" << std::endl;;
			// copy input data;
			const uchar *pSrc = (const uchar*)mat.data;
			// create QImage with same dimensions as input Mat
			QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
			return image.rgbSwapped();
		}
		else if (mat.type() == CV_8UC4)
		{
			std::cout << "CV_8UC4" << std::endl;;
			// copy input data;
			const uchar *pSrc = (const uchar*)mat.data;
			// create QImage with some dimension as input Mat
			QImage image(pSrc, mat.cols, mat.rows, mat.step, QImage::Format_ARGB32);
			return image.copy();
		}
		else
		{
			std::cerr << "ERROR: Mat could not be converted to QImage." << std::endl;;
			return QImage();
		}
	}
};
#endif