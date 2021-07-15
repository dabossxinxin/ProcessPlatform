#ifndef MYPOINTCLOUD_H
#define MYPOINTCLOUD_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXY Point2D;
typedef pcl::PointXYZ Point3D;
typedef pcl::PointCloud<Point3D> Cloud3D;
typedef pcl::PointCloud<Point3D>::Ptr Cloud3DPtr;

class MyPointCloud
{
public:
	MyPointCloud() {};
	~MyPointCloud() {};

	Cloud3DPtr mCloudPtr;
	std::string mFileName;
	std::string mSuffix;
	bool mVisible = true;
};
#endif