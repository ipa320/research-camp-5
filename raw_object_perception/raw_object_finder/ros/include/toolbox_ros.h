/*
 *  CToolBox.h
 *
 *  Created on: 09.12.2010
 *      Author: Christian Mueller
 */

#ifndef CTOOLBOXROS_H
#define CTOOLBOXROS_H

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include <ros/ros.h>
#include "sensor_msgs/Image.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

#include "pcl/ModelCoefficients.h"

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"

#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

#include "pcl/common/common_headers.h"
#include "pcl/range_image/range_image.h"

#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/surface/mls.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl_ros/segmentation/extract_clusters.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/features/normal_3d.h"
#include "pcl/surface/mls.h"
#include "pcl/segmentation/extract_polygonal_prism_data.h"

#include "struct_planar_surface.h" //since we need the structPlanarSurface
//template <typename pointCloud>


//#define EIGEN_DONT_ALIGN_STATICALLY

class CToolBoxROS {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	bool subsampling(pcl::PointCloud<pcl::PointXYZ> &cloud,
			const double dLeafsize);
	bool subsampling(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			const double dLeafsize);
	bool subsampling(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud,
			const double dLeafsize);

	bool filterDistance(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			float thresholdDistance);

	pcl::PointCloud<pcl::PointXYZ> filterDistance(
			pcl::PointCloud<pcl::PointXYZ> &cloud, float minDist,
			float maxDist, std::string axis);
	pcl::PointCloud<pcl::PointXYZRGB> filterDistance(pcl::PointCloud<
			pcl::PointXYZRGB> &cloud, float minDist, float maxDist,
			std::string axis);
	bool statisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
			int meanK, float stddevMulThresh);

	pcl::PointCloud<pcl::PointXYZRGBNormal> movingLeastSquares(pcl::PointCloud<
			pcl::PointXYZRGB> &cloud, float searchRadius = 0.005f);
	pcl::PointCloud<pcl::PointNormal> movingLeastSquares(pcl::PointCloud<
			pcl::PointXYZ> &cloud, float searchRadius);

	pcl::PointCloud<pcl::Normal> estimatingNormals(pcl::PointCloud<
			pcl::PointXYZRGB> &cloud, int KSearch = 50);
	//	pcl::PointCloud<pcl::FPFHSignature33> FPFHFeatureExtractor(pcl::PointCloud<pcl::PointXYZ> &cloud, float searchRadius);

	void markClusteredPointCloud(std::vector<pcl::PointCloud<
			pcl::PointXYZRGBNormal> > &clusteredPointCloud, pcl::PointCloud<
			pcl::PointXYZRGBNormal> &markedPointCloud);

	void markClusteredPointCloud(
			std::vector<pcl::PointCloud<pcl::PointXYZ> > &clusteredPointCloud,
			pcl::PointCloud<pcl::PointXYZ> &markedPointCloud);

	bool transformPointCloud(tf::TransformListener &tfListener,
			std::string &fromFrame, std::string &toFrame,
			const sensor_msgs::PointCloud2 &srcPointCloud,
			sensor_msgs::PointCloud2 &transformedPointCloud);

	int pointInsideConvexHull2d(
			pcl::PointCloud<pcl::PointXYZRGBNormal> convexHull,
			pcl::PointXYZRGBNormal point);
	int pointInsideConvexHull2d(
			pcl::PointCloud<pcl::PointXYZRGBNormal> convexHull,
			pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud);

	pcl::PointXYZRGBNormal centroidHull2d(pcl::PointCloud<
			pcl::PointXYZRGBNormal> point_cloud, float area);
	float areaConvexHull2d(pcl::PointCloud<pcl::PointXYZRGBNormal> hull);
	float avgValuePointCloud3d(
			pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud, int axis);
	float minValuePointCloud3d(
			pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud, int axis);
	float minValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> point_cloud,
			int axis);
	float minValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> point_cloud,
			int axis, pcl::PointXYZ &min_point);
	float maxValuePointCloud3d(
			pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud, int axis);
	float maxValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> point_cloud,
			int axis);
	float maxValuePointCloud3d(pcl::PointCloud<pcl::PointXYZ> point_cloud,
			int axis, pcl::PointXYZ &max_point);

	int overlapConvexHull2d(structPlanarSurface &surface1,
			structPlanarSurface &surface2);
	int overlapConvexHull2d2(structPlanarSurface &surface1,
			structPlanarSurface &surface2);

	bool distanceBetweenPlane2d(structPlanarSurface &surface1,
			structPlanarSurface &surface2, float distanceThreshold);
	bool isObjectPlane(structPlanarSurface &surface, pcl::PointCloud<
			pcl::PointXYZRGBNormal> object, float objectHeightThreshold,
			float objectPlaneHeightDifferenceThreshold);

	pcl::PointCloud<pcl::Normal> filterNormals(pcl::PointCloud<
			pcl::PointXYZRGBNormal> &cloud);

	pcl::PointCloud<pcl::PointXYZ> normalizePointCloud(pcl::PointCloud<
			pcl::PointXYZ> point_cloud);

	pcl::PointCloud<pcl::Boundary> estimatingBoundaryPoints(pcl::PointCloud<
			pcl::PointXYZRGB> &cloud);

	pcl::PointXYZ
	pointCloudCentroid(pcl::PointCloud<pcl::PointXYZ> point_cloud);
	pcl::PointXYZ pointCloudCentroid(
			pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud);

	std::vector<double> normalizePoint3D(std::vector<double> point);

	float angleBetweenPoints(std::vector<double> point1,
			std::vector<double> point2);

	void get3DPointsWithinHull(
			const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudPCLFull,
			const pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudPCLHull,
			const double dMinHeight, const double dMaxHeight, pcl::PointCloud<
					pcl::PointXYZRGBNormal> &cloudPCLSegmentOutput);

	pcl::PointXYZRGBNormal getNearestNeighborPlane(structPlanarSurface &plane,
			pcl::PointXYZRGBNormal queryPoint);

	double euclDistanceBtwPoints(pcl::PointXYZ p1, pcl::PointXYZ p2);
};

#endif

