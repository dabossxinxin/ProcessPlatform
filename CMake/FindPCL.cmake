
FIND_PATH(PCL_INCLUDE_DIR NAMES pcl-1.9 HINTS E:/MyFile/SDK/PCL-1.9.1/include)
FIND_PATH(PCL_LIB_DIR NAMES pcl_common_release.lib HINTS E:/MyFile/SDK/PCL-1.9.1/lib)
#FIND_PATH(PCL3RDPARTY_BOOST_INCLUDE_DIR NAMES boost-1_68 HINTS E:/MyFile/SDK/PCL-1.9.1/3rdParty/Boost/include/boost-1_68)
#FIND_PATH(PCL3RDPARTY_EIGEN_INCLUDE_DIR NAMES Eigen HINTS E:/MyFile/SDK/PCL-1.9.1/3rdParty/Eigen/eigen3)
#FIND_PATH(PCL3RDPARTY_FLANN_INCLUDE_DIR NAMES flann.h HINTS E:/MyFile/SDK/PCL-1.9.1/3rdParty/FLANN/include/flann)
#FIND_PATH(PCL3RDPARTY_OPENNI2_INCLUDE_DIR NAMES OpenNI.h HINTS E:/MyFile/SDK/PCL-1.9.1/3rdParty/OpenNI2/Include)
#FIND_PATH(PCL3RDPARTY_QHULL_INCLUDE_DIR NAMES libqhull HINTS E:/MyFile/SDK/PCL-1.9.1/3rdParty/Qhull/include)
#FIND_PATH(PCL3RDPARTY_VTK_INCLUDE_DIR NAMES pcl-1.9 HINTS E:/MyFile/SDK/PCL-1.9.1/3rdParty/Boost/include/boost-1_68)

SET(PCL_LIBRARIES
	pcl_common_
	pcl_features_
	pcl_filters_
	pcl_io_ply_
	pcl_io_
	pcl_kdtree_
	pcl_keypoints_
	pcl_ml_
	pcl_octree_
	pcl_outofcore_
	pcl_people_
	pcl_recognition_
	pcl_registration_
	pcl_sample_consensus_
	pcl_search_
	pcl_segmentation_
	pcl_stereo_
	pcl_surface_
	pcl_tracking_
	pcl_visualization_
	)
	
	

 SET(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIR}/pcl-1.9)

