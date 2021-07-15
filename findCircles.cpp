#include "findCircles.h"

int CoreAlgorithm::disPnt(cv::Point2i srcPnt, cv::Point2i dstPnt)
{
	int dis;
	dis = pow((srcPnt.x - dstPnt.x), 2) + pow((srcPnt.y-dstPnt.y),2);
	return dis;
}

