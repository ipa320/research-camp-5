/*
 *  CToolBox.cpp
 *
 *  Created on: 09.12.2010
 *      Author: Christian Mueller
 */

#include "toolbox_ros.h"

double CToolBoxROS::euclDistanceBtwPoints(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
	double x, y, z;

	x = pow((p1.x - p2.x),2);
	y = pow((p1.y - p2.y),2);
	z = pow((p1.z - p2.z),2);

	return sqrt(x+y+z);
}
pcl::PointCloud<pcl::PointXYZ> CToolBoxROS::filterDistance(pcl::PointCloud<
		pcl::PointXYZ> &cloud, float minDist, float maxDist, std::string axis) {
	pcl::PassThrough<pcl::PointXYZ> pass;

	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

	pass.setInputCloud(cloud.makeShared());
	pass.setFilterFieldName(axis);
	pass.setFilterLimits(minDist, maxDist);

	pass.filter(cloud_filtered);

	return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB> CToolBoxROS::filterDistance(pcl::PointCloud<
		pcl::PointXYZRGB> &cloud, float minDist, float maxDist,
		std::string axis) {
	pcl::PassThrough<pcl::PointXYZRGB> pass;

	pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

//	pass.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud));
	pass.setInputCloud(cloud.makeShared());

	pass.setFilterFieldName(axis);
	pass.setFilterLimits(minDist, maxDist);

	pass.filter(cloud_filtered);

	return cloud_filtered;
}

//spherical distance filtering
bool CToolBoxROS::filterDistance(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
		float thresholdDistance) {
	float pointDistance;
	pcl::PointCloud<pcl::PointXYZRGB> filteredDistanceCloud;

	pcl::PointIndices filteredDistanceIndices;
	for (size_t i = 0; i < (cloud.points.size()); i++) {
		pointDistance = sqrt(pow(fabs(cloud.points[i].x), 2) + pow(fabs(
				cloud.points[i].y), 2) + pow(fabs(cloud.points[i].z), 2));

		if (pointDistance > thresholdDistance) {
			filteredDistanceIndices.indices.push_back(i);
		}
	}

	if (filteredDistanceIndices.indices.size() > 0) {
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud(cloud.makeShared());
		extract.setIndices(boost::make_shared<pcl::PointIndices>(filteredDistanceIndices));
		extract.setNegative(true);
		extract.filter(filteredDistanceCloud);

		cloud.width = filteredDistanceCloud.width;
		cloud.height = filteredDistanceCloud.height;
		cloud.points = filteredDistanceCloud.points;
		//filteredDistanceIndices.indices.clear();
		return true;
	}

	return false;
}

bool CToolBoxROS::subsampling(pcl::PointCloud<pcl::PointXYZ> &cloud,
		const double dLeafsize) {
	pcl::PointCloud<pcl::PointXYZ> cloud_filtered;

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud.makeShared());
	sor.setLeafSize(dLeafsize, dLeafsize, dLeafsize);
	sor.filter(cloud_filtered);
	cloud = cloud_filtered;

	return false;
}

bool CToolBoxROS::subsampling(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
		const double dLeafsize) {
	pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud.makeShared());
	sor.setLeafSize(dLeafsize, dLeafsize, dLeafsize);
	sor.filter(cloud_filtered);
	cloud = cloud_filtered;

	return false;
}

bool CToolBoxROS::subsampling(pcl::PointCloud<pcl::PointXYZRGBNormal> &cloud,
		const double dLeafsize) {
	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_filtered;

	pcl::VoxelGrid<pcl::PointXYZRGBNormal> sor;
//	sor.setInputCloud(boost::make_shared< pcl::PointCloud<pcl::PointXYZRGBNormal> >(cloud));
  sor.setInputCloud(cloud.makeShared());
	sor.setInputCloud(cloud.makeShared());
	sor.setLeafSize(dLeafsize, dLeafsize, dLeafsize);
	sor.filter(cloud_filtered);
	cloud = cloud_filtered;

	return false;
}

bool CToolBoxROS::statisticalOutlierRemoval(
		pcl::PointCloud<pcl::PointXYZRGB> &cloud, int meanK,
		float stddevMulThresh) {
	pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(cloud.makeShared());
	sor.setMeanK(meanK);
	sor.setStddevMulThresh(stddevMulThresh);
	sor.filter(cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << cloud_filtered << std::endl;

	sor.setNegative(false); //get inliers = true
	sor.filter(cloud_filtered);

	cloud = cloud_filtered;

	return false;
}

pcl::PointCloud<pcl::PointXYZRGBNormal> CToolBoxROS::movingLeastSquares(
		pcl::PointCloud<pcl::PointXYZRGB> &cloud, float searchRadius) {
	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<
			pcl::KdTreeFLANN<pcl::PointXYZRGB> >();
	pcl::PointCloud<pcl::PointXYZRGB> mls_points;
	pcl::PointCloud<pcl::Normal>::Ptr mls_normals(new pcl::PointCloud<pcl::Normal> ());

	tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> >();
	tree->setInputCloud(cloud.makeShared());

	pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::Normal>
			movingLeastSquaresExtractor;
	//movingLeastSquaresExtractor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(cloud));
  movingLeastSquaresExtractor.setInputCloud(cloud.makeShared());
	movingLeastSquaresExtractor.setOutputNormals(mls_normals);
	movingLeastSquaresExtractor.setPolynomialFit(true);
	movingLeastSquaresExtractor.setSearchRadius(searchRadius);
	movingLeastSquaresExtractor.setPolynomialOrder(3);
	movingLeastSquaresExtractor.setSearchMethod(tree);
	movingLeastSquaresExtractor.reconstruct(mls_points);
	
	pcl::PointCloud<pcl::PointXYZRGBNormal> mls_cloud;
  	pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);

	return mls_cloud;
}

pcl::PointCloud<pcl::PointNormal> CToolBoxROS::movingLeastSquares(
		pcl::PointCloud<pcl::PointXYZ> &cloud, float searchRadius) {
		pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree = boost::make_shared<
			pcl::KdTreeFLANN<pcl::PointXYZ> >();
	pcl::PointCloud<pcl::PointXYZ> mls_points;
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_normals(new pcl::PointCloud<pcl::PointNormal> ());

	tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> >();
	tree->setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(
			cloud));

	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>
			movingLeastSquaresExtractor;
	movingLeastSquaresExtractor.setInputCloud(boost::make_shared<
			pcl::PointCloud<pcl::PointXYZ> >(cloud));
	movingLeastSquaresExtractor.setOutputNormals(mls_normals);
	movingLeastSquaresExtractor.setPolynomialFit(true);
	movingLeastSquaresExtractor.setSearchRadius(searchRadius);
	movingLeastSquaresExtractor.setPolynomialOrder(3);
	movingLeastSquaresExtractor.setSearchMethod(tree);
	movingLeastSquaresExtractor.reconstruct(mls_points);

	return *mls_normals;
}
/*
 bool CToolBox::movingLeastSquares2(pcl::PointCloud<pcl::PointXYZ> &cloud, float searchRadius)
 {
 pcl::PointCloud<pcl::PointNormal>  cloud_normals;
 pcl::PointCloud<pcl::PointXYZ> point_cloud_seg;

 cloud_normals = this->movingLeastSquares(cloud,searchRadius,3);

 cloud.points.clear();
 cloud.width= cloud_normals.width;
 cloud.height= cloud_normals.height;
 cloud.points.resize(cloud_normals.points.size());

 for(unsigned int iterPoint= 0;iterPoint < cloud_normals.points.size(); iterPoint++)
 {
 cloud.points.at(iterPoint).x = (float)cloud_normals.points.at(iterPoint).x;
 cloud.points.at(iterPoint).y = (float)cloud_normals.points.at(iterPoint).y;
 cloud.points.at(iterPoint).z = (float)cloud_normals.points.at(iterPoint).z;
 }
 return false;
 }
 */
pcl::PointCloud<pcl::Normal> CToolBoxROS::estimatingNormals(pcl::PointCloud<
		pcl::PointXYZRGB> &cloud, int KSearch) {
	pcl::PointCloud<pcl::Normal> cloud_normals;

	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree = boost::make_shared<
			pcl::KdTreeFLANN<pcl::PointXYZRGB> >();
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;

	normalEstimation.setSearchMethod(tree);
	normalEstimation.setInputCloud(boost::make_shared<pcl::PointCloud<
			pcl::PointXYZRGB> >(cloud));

	if (KSearch != -1)
		normalEstimation.setKSearch(KSearch);
	//if(searchRadius!=-1)
	//normalEstimation.setRadiusSearch(searchRadius);
	normalEstimation.compute(cloud_normals);

	return cloud_normals;

}

pcl::PointCloud<pcl::Boundary> CToolBoxROS::estimatingBoundaryPoints(
		pcl::PointCloud<pcl::PointXYZRGB> &cloud) {

	pcl::PointCloud<pcl::Boundary> cloud_boundary;
	std::cout << "not implemented";
	/*
	 pcl::Boundary<pcl::PointXYZRGB, pcl::pcl::Boundary> normalEstimation;

	 normalEstimation.setSearchMethod(tree);
	 normalEstimation.setInputCloud(boost::make_shared<pcl::PointCloud<
	 pcl::PointXYZRGB> >(cloud));

	 if (KSearch != -1)
	 normalEstimation.setKSearch(KSearch);
	 //if(searchRadius!=-1)
	 //normalEstimation.setRadiusSearch(searchRadius);
	 normalEstimation.compute(cloud_normals);*/

	return cloud_boundary;

}

pcl::PointCloud<pcl::Normal> CToolBoxROS::filterNormals(pcl::PointCloud<
		pcl::PointXYZRGBNormal> &cloud) {
	pcl::PointCloud<pcl::Normal> cloud_normals;

	cloud_normals.points.resize(cloud.points.size());
	for (unsigned int i = 0; i < cloud.points.size(); i++) {
		cloud_normals.points[i].normal[0] = cloud.points[i].normal[0];
		cloud_normals.points[i].normal[1] = cloud.points[i].normal[1];
		cloud_normals.points[i].normal[2] = cloud.points[i].normal[2];
	}

	return cloud_normals;

}
/*
 pcl::PointCloud<pcl::FPFHSignature33> CToolBox::FPFHFeatureExtractor(pcl::PointCloud<pcl::PointXYZ> &cloud, float searchRadius)
 {
 pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
 pcl::PointCloud<pcl::Normal> normalEstimation;
 pcl::PointCloud<pcl::FPFHSignature33> cloudFeatures;
 pcl::KdTreeFLANN<pcl::PointXYZ> tree;

 fpfhEstimation.setSearchMethod(boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > (tree));
 fpfhEstimation.setRadiusSearch (searchRadius);

 normalEstimation = this->estimatingNormals(cloud,0,searchRadius);

 fpfhEstimation.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud));
 fpfhEstimation.setInputNormals(boost::make_shared<pcl::PointCloud<pcl::Normal> > (normalEstimation));
 fpfhEstimation.compute(cloudFeatures);

 return cloudFeatures;
 }*/

void CToolBoxROS::markClusteredPointCloud(std::vector<pcl::PointCloud<
		pcl::PointXYZRGBNormal> > &clusteredPointCloud, pcl::PointCloud<
		pcl::PointXYZRGBNormal> &markedPointCloud) {
	int color = 0;

	if (clusteredPointCloud.size() > 0) {
		markedPointCloud.points.clear();

		for (unsigned int iterCluster = 0; iterCluster
				< clusteredPointCloud.size(); iterCluster++) {
			color = rand() % 10000;
			for (unsigned int iterPoint = 0; iterPoint
					< clusteredPointCloud.at(iterCluster).points.size(); iterPoint++) {
				pcl::PointXYZRGBNormal pointRGB;
				pointRGB = clusteredPointCloud.at(iterCluster).points.at(
						iterPoint);
				pointRGB.rgb = color;

				markedPointCloud.points.push_back(pointRGB);
			}
		}

		markedPointCloud.header.frame_id
				= clusteredPointCloud.at(0).header.frame_id;
		markedPointCloud.height = markedPointCloud.points.size();
		markedPointCloud.width = 1;
	}
}

void CToolBoxROS::markClusteredPointCloud(std::vector<pcl::PointCloud<
		pcl::PointXYZ> > &clusteredPointCloud,
		pcl::PointCloud<pcl::PointXYZ> &markedPointCloud) {
	int color = 0;

	if (clusteredPointCloud.size() > 0) {
		markedPointCloud.points.clear();

		for (unsigned int iterCluster = 0; iterCluster
				< clusteredPointCloud.size(); iterCluster++) {
			color = rand() % 10000;
			for (unsigned int iterPoint = 0; iterPoint
					< clusteredPointCloud.at(iterCluster).points.size(); iterPoint++) {
				pcl::PointXYZ pointRGB;
				pointRGB = clusteredPointCloud.at(iterCluster).points.at(
						iterPoint);
				markedPointCloud.points.push_back(pointRGB);
			}
		}

		markedPointCloud.header.frame_id
				= clusteredPointCloud.at(0).header.frame_id;
		markedPointCloud.height = markedPointCloud.points.size();
		markedPointCloud.width = 1;
	}
}

bool CToolBoxROS::transformPointCloud(tf::TransformListener &tfListener,
		std::string &fromFrame, std::string &toFrame,
		const sensor_msgs::PointCloud2 &srcPointCloud,
		sensor_msgs::PointCloud2 &transformedPointCloud) {

	//bool setup_tf = true;
	bool success_tf = false;
	tf::StampedTransform transform;

	sensor_msgs::PointCloud pointCloudMsgConvert;
	sensor_msgs::PointCloud pointCloudMsgTransformed;

	sensor_msgs::convertPointCloud2ToPointCloud(srcPointCloud,
			pointCloudMsgConvert);

	//while (setup_tf) {
		//setup_tf = false;
		try {
		// tfListener.waitForTransform(fromFrame, toFrame, ros::Time(0),
		tfListener.waitForTransform(fromFrame, toFrame, ros::Time::now(),
					ros::Duration(1.0));
		tfListener.transformPointCloud(std::string(toFrame),
					pointCloudMsgConvert, pointCloudMsgTransformed);
		success_tf = true;
		} catch (tf::TransformException ex) {
		//	setup_tf = true;
			success_tf = false;
			return success_tf;
		}
	//}
	//setup_tf = true;

	sensor_msgs::convertPointCloudToPointCloud2(pointCloudMsgTransformed,
			transformedPointCloud);

	return success_tf;
}

//1 for interior points and 0 for exterior points
int CToolBoxROS::pointInsideConvexHull2d(
		pcl::PointCloud<pcl::PointXYZRGBNormal> convexHull,
		pcl::PointXYZRGBNormal point) {
	unsigned int convexHullSize = convexHull.points.size();

	unsigned int i, j, c = 0;
	for (i = 0, j = convexHullSize - 1; i < convexHullSize; j = i++) {
		if ((((convexHull.points[i].y <= point.y) && (point.y
				< convexHull.points[j].y)) || ((convexHull.points[j].y
				<= point.y) && (point.y < convexHull.points[i].y))) && (point.x
				< (convexHull.points[j].x - convexHull.points[i].x) * (point.y
						- convexHull.points[i].y) / (convexHull.points[j].y
						- convexHull.points[i].y) + convexHull.points[i].x))

			c = !c;
	}
	return c;
}

//1 for interior points and 0 for exterior points
int CToolBoxROS::pointInsideConvexHull2d(
		pcl::PointCloud<pcl::PointXYZRGBNormal> convexHull, pcl::PointCloud<
				pcl::PointXYZRGBNormal> point_cloud) {
	for (unsigned int iter = 0; iter < point_cloud.points.size(); iter++) {
		if (this->pointInsideConvexHull2d(convexHull, point_cloud.points[iter]))
			return 1;
	}
	return 0;
}

//low computation comparison based on centroid
int CToolBoxROS::overlapConvexHull2d(structPlanarSurface &surface1,
		structPlanarSurface &surface2) {

	structPlanarSurface surfaceLarger;
	structPlanarSurface surfaceSmaller;
	//find the largest surface
	if (surface1.area > surface2.area) {
		surfaceLarger = surface1;
		surfaceSmaller = surface2;
	} else {
		surfaceLarger = surface2;
		surfaceSmaller = surface1;
	}

	return this->pointInsideConvexHull2d(surfaceLarger.convexHull,
			surfaceSmaller.centroid);

}

pcl::PointXYZRGBNormal CToolBoxROS::getNearestNeighborPlane(structPlanarSurface &plane, pcl::PointXYZRGBNormal queryPoint)
{
//	std::cout<<"ENTER";
	int k=1;
	std::vector<int> k_indicies;
	std::vector<float> k_distances;

	k_distances.resize(k);
    k_indicies.resize(k);
	plane.tree->nearestKSearch(
			queryPoint, k, k_indicies,
					k_distances);


	return plane.pointCloud.points[k_indicies[0]];
}

//computationally costly
int CToolBoxROS::overlapConvexHull2d2(structPlanarSurface &surface1,
		structPlanarSurface &surface2) {
	structPlanarSurface surfaceLarger;
	structPlanarSurface surfaceSmaller;
	//find the largest surface
	if (surface1.area > surface2.area) {
		surfaceLarger = surface1;
		surfaceSmaller = surface2;
	} else {
		surfaceLarger = surface2;
		surfaceSmaller = surface1;
	}

	//the largest surface is used to be checked whether the small one is intersecting the larger one
	int k = 1;
	std::vector<int> k_indicies;
	std::vector<float> k_distances;
	for (unsigned int iter = 0; iter < surfaceLarger.convexHull.points.size(); iter++) {

		k_distances.clear();
		k_indicies.clear();

		k_distances.resize(k);
		k_indicies.resize(k);
		//find from smaller surfaces the nearstest point to a convexHull point of the larger surface.
		//the nearest point is used to be checked whether the point is inside the large surface convex hull
		//if so there is a overlap! -> return 1
		surfaceSmaller.tree->nearestKSearch(
				surfaceLarger.convexHull.points[iter], k, k_indicies,
				k_distances);

		if (this->pointInsideConvexHull2d(surfaceLarger.convexHull,
				surfaceSmaller.pointCloud.points[k_indicies[0]])) {
			return 1;
		}
	}
	return 0;
}

//non-self-intersecting polygon
float CToolBoxROS::areaConvexHull2d(
		pcl::PointCloud<pcl::PointXYZRGBNormal> hull) {
	unsigned int sizeHull = hull.points.size();
	float area = 0;

	for (unsigned int iter_point = 0; iter_point < sizeHull - 2; ++iter_point)
	{
		area += (hull.points[iter_point].x * hull.points[iter_point + 1].y - hull.points[iter_point + 1].x * hull.points[iter_point].y);
	}
	if (sizeHull > 0)
	{
		area += (hull.points[sizeHull - 1].x * hull.points[0].y - hull.points[0].x * hull.points[sizeHull - 1].y);
	}

	return 0.5f * area;
}
pcl::PointXYZ CToolBoxROS::pointCloudCentroid(
		pcl::PointCloud<pcl::PointXYZ> point_cloud) {

	unsigned int sizePointCloud = point_cloud.points.size();

	pcl::PointXYZ centroid;
	centroid.x = 0;
	centroid.y = 0;
	centroid.z = 0;

	for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
		centroid.x += (point_cloud.points[iter_point].x);
		centroid.y += (point_cloud.points[iter_point].y);
		centroid.z += (point_cloud.points[iter_point].z);
	}

	centroid.x /= sizePointCloud;
	centroid.y /= sizePointCloud;
	centroid.z /= sizePointCloud;

	return centroid;
}

pcl::PointXYZ CToolBoxROS::pointCloudCentroid(
		pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud) {

	unsigned int sizePointCloud = point_cloud.points.size();

	pcl::PointXYZ centroid;
	centroid.x = 0;
	centroid.y = 0;
	centroid.z = 0;

	for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
		centroid.x += (point_cloud.points[iter_point].x);
		centroid.y += (point_cloud.points[iter_point].y);
		centroid.z += (point_cloud.points[iter_point].z);
	}

	centroid.x /= sizePointCloud;
	centroid.y /= sizePointCloud;
	centroid.z /= sizePointCloud;

	return centroid;
}

//non-self-intersecting polygon
pcl::PointXYZRGBNormal CToolBoxROS::centroidHull2d(pcl::PointCloud<
		pcl::PointXYZRGBNormal> point_cloud, float area) {
	unsigned int sizePointCloud = point_cloud.points.size();

	pcl::PointXYZRGBNormal centroid;
	centroid.x = 0;
	centroid.y = 0;
	centroid.z = 0;

	for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
		centroid.z += point_cloud.points[iter_point].z;
	}
	centroid.z /= sizePointCloud;

	for (unsigned int iter_point = 0; iter_point < sizePointCloud - 1; iter_point++) {
		centroid.x += (point_cloud.points[iter_point].x
				+ point_cloud.points[iter_point + 1].x)
				* (point_cloud.points[iter_point].x
						* point_cloud.points[iter_point + 1].y
						- point_cloud.points[iter_point + 1].x
								* point_cloud.points[iter_point].y);
		centroid.y += (point_cloud.points[iter_point].y
				+ point_cloud.points[iter_point + 1].y)
				* (point_cloud.points[iter_point].x
						* point_cloud.points[iter_point + 1].y
						- point_cloud.points[iter_point + 1].x
								* point_cloud.points[iter_point].y);
	}

	centroid.x = centroid.x / (6 * area);
	centroid.y = centroid.y / (6 * area);

	return centroid;
}

float CToolBoxROS::avgValuePointCloud3d(
		pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud, int axis) {
	unsigned int sizePointCloud = point_cloud.points.size();
	float sumValue = 0;
	if (axis == 0) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			sumValue += point_cloud.points[iter_point].x;
		}

		return sumValue / sizePointCloud;
	}

	if (axis == 1) {
		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			sumValue += point_cloud.points[iter_point].y;
		}

		return sumValue / sizePointCloud;
	}

	if (axis == 2) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			sumValue += point_cloud.points[iter_point].z;
		}

		return sumValue / sizePointCloud;
	}

	return 0;
}

float CToolBoxROS::minValuePointCloud3d(
		pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud, int axis) {
	unsigned int sizePointCloud = point_cloud.points.size();
	float minValue = 9999;
	if (axis == 0) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].x < minValue)
				minValue = point_cloud.points[iter_point].x;
		}

		return minValue;
	}

	if (axis == 1) {
		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].y < minValue)
				minValue = point_cloud.points[iter_point].y;
		}

		return minValue;
	}

	if (axis == 2) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].z < minValue)
				minValue = point_cloud.points[iter_point].z;
		}

		return minValue;
	}

	return 0;
}

float CToolBoxROS::minValuePointCloud3d(
		pcl::PointCloud<pcl::PointXYZ> point_cloud, int axis) {
	unsigned int sizePointCloud = point_cloud.points.size();
	float minValue = 9999;
	if (axis == 0) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].x < minValue)
				minValue = point_cloud.points[iter_point].x;
		}

		return minValue;
	}

	if (axis == 1) {
		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].y < minValue)
				minValue = point_cloud.points[iter_point].y;
		}

		return minValue;
	}

	if (axis == 2) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].z < minValue)
				minValue = point_cloud.points[iter_point].z;
		}

		return minValue;
	}

	return 0;
}

float CToolBoxROS::minValuePointCloud3d(
		pcl::PointCloud<pcl::PointXYZ> point_cloud, int axis,
		pcl::PointXYZ &min_point) {
	unsigned int sizePointCloud = point_cloud.points.size();
	float minValue = 9999;
	if (axis == 0) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].x < minValue) {
				minValue = point_cloud.points[iter_point].x;
				min_point = point_cloud.points[iter_point];
			}
		}

		return minValue;
	}

	if (axis == 1) {
		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].y < minValue) {
				minValue = point_cloud.points[iter_point].y;
				min_point = point_cloud.points[iter_point];
			}
		}

		return minValue;
	}

	if (axis == 2) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].z < minValue) {
				minValue = point_cloud.points[iter_point].z;
				min_point = point_cloud.points[iter_point];
			}
		}

		return minValue;
	}

	return 0;
}

float CToolBoxROS::maxValuePointCloud3d(
		pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud, int axis) {
	unsigned int sizePointCloud = point_cloud.points.size();
	float maxValue = -999999;
	if (axis == 0) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].x > maxValue)
				maxValue = point_cloud.points[iter_point].x;
		}

		return maxValue;
	}

	if (axis == 1) {
		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].y > maxValue)
				maxValue = point_cloud.points[iter_point].y;
		}

		return maxValue;
	}

	if (axis == 2) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].z > maxValue)
				maxValue = point_cloud.points[iter_point].z;
		}

		return maxValue;
	}

	return 0;
}
float CToolBoxROS::maxValuePointCloud3d(
		pcl::PointCloud<pcl::PointXYZ> point_cloud, int axis,
		pcl::PointXYZ &max_point) {
	unsigned int sizePointCloud = point_cloud.points.size();
	float maxValue = -999999;
	if (axis == 0) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].x > maxValue) {
				maxValue = point_cloud.points[iter_point].x;
				max_point = point_cloud.points[iter_point];
			}
		}

		return maxValue;
	}

	if (axis == 1) {
		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].y > maxValue) {
				maxValue = point_cloud.points[iter_point].y;
				max_point = point_cloud.points[iter_point];
			}
		}

		return maxValue;
	}

	if (axis == 2) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].z > maxValue) {
				maxValue = point_cloud.points[iter_point].z;
				max_point = point_cloud.points[iter_point];
			}
		}

		return maxValue;
	}

	return 0;
}

float CToolBoxROS::maxValuePointCloud3d(
		pcl::PointCloud<pcl::PointXYZ> point_cloud, int axis) {
	unsigned int sizePointCloud = point_cloud.points.size();
	float maxValue = -999999;
	if (axis == 0) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].x > maxValue)
				maxValue = point_cloud.points[iter_point].x;
		}

		return maxValue;
	}

	if (axis == 1) {
		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].y > maxValue)
				maxValue = point_cloud.points[iter_point].y;
		}

		return maxValue;
	}

	if (axis == 2) {

		for (unsigned int iter_point = 0; iter_point < sizePointCloud; iter_point++) {
			if (point_cloud.points[iter_point].z > maxValue)
				maxValue = point_cloud.points[iter_point].z;
		}

		return maxValue;
	}

	return 0;
}

// low computational cost required: we take the smaller plane then we search for the closest point from the that plane to the other(larger plane)
bool CToolBoxROS::distanceBetweenPlane2d(structPlanarSurface &surface1,
		structPlanarSurface &surface2, float distanceThreshold) {
	//ROS_INFO("[ToolBox]DistanceBetweenPlane2d started...");
	ros::Time start, finish;
	start = ros::Time::now();
	//float distance;
	float minDistance = 5; //since 5m is max range of kinect, however ToDo limit.h
	structPlanarSurface surfaceLarger;
	structPlanarSurface surfaceSmaller;
	//find the largest surface
	if (surface1.pointCloud.size() > surface2.pointCloud.size()) {
		surfaceLarger = surface1;
		surfaceSmaller = surface2;
	} else {
		surfaceLarger = surface2;
		surfaceSmaller = surface1;
	}

	int k = 1;
	std::vector<int> k_indicies;
	std::vector<float> k_distances;
	for (unsigned int iter = 0; iter < surfaceSmaller.pointCloud.points.size(); iter++) {

		k_distances.clear();
		k_indicies.clear();

		k_distances.resize(k);
		k_indicies.resize(k);

		surfaceLarger.tree->nearestKSearch(
				surfaceSmaller.pointCloud.points[iter], k, k_indicies,
				k_distances);

		if (k_distances[0] <= minDistance) {
			minDistance = k_distances[0]; //0 since we are looking for the nearest one!
		}
	}
	finish = ros::Time::now();
	//	ROS_INFO("...distanceBetweenPlane2d(%f) took %lf", minDistance, (finish.toSec() - start.toSec() ));
	return (minDistance < distanceThreshold);
}

//checks whether an "object" is just a planar surface which is not that interesting, if we a just interested in objects
bool CToolBoxROS::isObjectPlane(structPlanarSurface &surface, pcl::PointCloud<
		pcl::PointXYZRGBNormal> object, float objectHeightThreshold,
		float objectPlaneHeightDifferenceThreshold) {

	float zMinObject = 5.0f; //min height of the object
	float zMaxObject = 0.0f;

	for (unsigned int iter_point = 0; iter_point < object.points.size(); iter_point++) {
		if (object.points[iter_point].z < zMinObject)
			zMinObject = object.points[iter_point].z;

		if (object.points[iter_point].z > zMaxObject)
			zMaxObject = object.points[iter_point].z;
	}

	//ROS_WARN("object height %f/%f ; diff %f/%f  ",fabs(zMaxObject-zMinObject),objectHeightThreshold,(zMinObject - surface.plane_height),objectPlaneHeightDifferenceThreshold);

	//Todo here we just look at "flying" objects, this is true for many cases, but what about flying labels of transparent bottles? They are still objects. in that case they are neglected!!
	//if(fabs(zMaxObject-zMinObject) > objectHeightThreshold)
	//	return false;

	if ((zMinObject - surface.plane_height)
			< objectPlaneHeightDifferenceThreshold)
		return false;

	return true;
}

pcl::PointCloud<pcl::PointXYZ> CToolBoxROS::normalizePointCloud(
		pcl::PointCloud<pcl::PointXYZ> point_cloud) {

	pcl::PointCloud<pcl::PointXYZ> point_cloudNorm;
	point_cloudNorm.points.resize(point_cloud.points.size());

	for (unsigned int iterPoints = 0; iterPoints < point_cloud.points.size(); iterPoints++) {
		double a = sqrt((point_cloud.points[iterPoints].x
				* point_cloud.points[iterPoints].x)
				+ (point_cloud.points[iterPoints].y
						* point_cloud.points[iterPoints].y)
				+ (point_cloud.points[iterPoints].z
						* point_cloud.points[iterPoints].z));

		point_cloudNorm.points[iterPoints].x = point_cloud.points[iterPoints].x
				/ a;
		point_cloudNorm.points[iterPoints].y = point_cloud.points[iterPoints].y
				/ a;
		point_cloudNorm.points[iterPoints].z = point_cloud.points[iterPoints].z
				/ a;

	}

	return point_cloudNorm;
}


std::vector<double> CToolBoxROS::normalizePoint3D(std::vector<double> point)
{
	std::vector<double> normalizedpoint;

	normalizedpoint.resize(point.size());
	double a;
	double sum = 0;

	for(unsigned int i = 0; i<point.size(); i++)
	{
		sum +=  (point[i] * point[i]);
	}

	a = sqrt(sum);

	for(unsigned int i = 0; i<point.size(); i++)
	{
		normalizedpoint[i] =  (point[i]/a);
	}

	return normalizedpoint;
}

float CToolBoxROS::angleBetweenPoints(std::vector<double> point1, std::vector<double> point2)
{
	float angle = 0;
	if(point1.size()!=point2.size())
		return angle;

	std::vector<double> normalizedpoint1;
	std::vector<double> normalizedpoint2;

	normalizedpoint1 = this->normalizePoint3D(point1);
	normalizedpoint2 = this->normalizePoint3D(point2);

	double sum = 0;
	for(unsigned int i = 0; i<normalizedpoint1.size(); i++)
	{
		sum += normalizedpoint1[i] +normalizedpoint2[i];
	}

	angle = acos(sum);

	return angle;
}

void CToolBoxROS::get3DPointsWithinHull(const pcl::PointCloud<
		pcl::PointXYZRGBNormal> &cloudPCLFull, const pcl::PointCloud<
		pcl::PointXYZRGBNormal> &cloudPCLHull, const double dMinHeight,
		const double dMaxHeight,
		pcl::PointCloud<pcl::PointXYZRGBNormal> &cloudPCLSegmentOutput) {
	pcl::PointIndices object_indices;
	pcl::ExtractPolygonalPrismData<pcl::PointXYZRGBNormal> hull_limiter;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPCLFullPtr(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	*cloudPCLFullPtr = cloudPCLFull;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudPCLHullPtr(
			new pcl::PointCloud<pcl::PointXYZRGBNormal>); // HERE SOMETIMES AND ERROR OCCURS !
	*cloudPCLHullPtr = cloudPCLHull;

	hull_limiter.setInputCloud(cloudPCLFullPtr);
	hull_limiter.setInputPlanarHull(cloudPCLHullPtr);
	hull_limiter.setHeightLimits(dMinHeight, dMaxHeight);
	hull_limiter.segment(object_indices);

	pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
	extract.setInputCloud(cloudPCLFullPtr);
	extract.setIndices(boost::make_shared<pcl::PointIndices>(object_indices));
	extract.setNegative(false);
	extract.filter(cloudPCLSegmentOutput);
}

