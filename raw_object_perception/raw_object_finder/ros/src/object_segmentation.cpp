#include "object_segmentation.h"
#include <raw_msgs/ObjectList.h>


ObjectSegmentation::ObjectSegmentation(const std::string &node_name)
{
	_node_name = node_name;

	ros::NodeHandle pn("~");

    pn.param("extract_obj_in_rgb_img", _extract_obj_in_rgb_img, false);
    ROS_INFO_STREAM("extract objects in rgb image: " << _extract_obj_in_rgb_img);

	pn.param("min_x", _dist_min_x, -1.5);
	pn.param("max_x", _dist_max_x,  1.5);
	pn.param("min_y", _dist_min_y, -1.5);
	pn.param("max_y", _dist_max_y,  1.5);
	pn.param("min_z", _dist_min_z,  0.5);
	pn.param("max_z", _dist_max_z,  1.5);
	pn.param("camera_frame", _camera_frame, std::string("/openni_rgb_optical_frame"));
	pn.param("downsampling_distance", _downsampling_distance, 0.02);
	ROS_INFO_STREAM("   parameter 'downsampling_distance': " << this->_downsampling_distance);

	double spherical_distance;
	pn.param("spherical_distance", spherical_distance, 2.5);

	_object_candidate_extractor = new CObjectCandidateExtraction(pn, node_name, spherical_distance);
	_roi_extractor = new RoiExtraction(_camera_frame);

	_objects_points_pub = pn.advertise<sensor_msgs::PointCloud2>("segmented_objects_points", 1);
	ROS_INFO("Publishing on 'segmented_objects_points' topic");

	_surfaces_points_pub = pn.advertise<sensor_msgs::PointCloud2>("segmented_surfaces_points", 1);
	ROS_INFO("Publishing on 'segmented_surfaces_points' topic");

	_objects_pub = pn.advertise<raw_msgs::ObjectList>("segmented_objects", 1);
	ROS_INFO("Publishing on 'segmented_objects' topic");

	_get_segmented_objects_srv = pn.advertiseService("get_segmented_objects", &ObjectSegmentation::GetObjects, this);
	ROS_INFO("Advertised 'get_segmented_objects' service");

	ROS_INFO("Object segmentation started");
}


ObjectSegmentation::~ObjectSegmentation()
{
	delete _object_candidate_extractor;
	delete _roi_extractor;
}

bool ObjectSegmentation::GetObjects(
		raw_srvs::GetObjects::Request &req,
		raw_srvs::GetObjects::Response &res)
{
	sensor_msgs::PointCloud2::ConstPtr const_input_cloud;
	sensor_msgs::PointCloud2 input_cloud;
	pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

	do
	{
		const_input_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points", _nh, ros::Duration(5));
		std::cout << "1" << std::endl;
		input_cloud = *const_input_cloud;

	}while(!PreparePointCloud(input_cloud, point_cloud));

	std::cout << "12" << std::endl;


	try {
		// start with an empty set of segmented objects
		_last_segmented_objects.objects.clear();


		// point cloud colors to color image
		IplImage *image = ClusterToImage(input_cloud);

		std::cout << "2" << std::endl;

		// find planes and objects
		pcl::PointCloud<pcl::PointXYZRGBNormal> planar_point_cloud;
		std::vector<structPlanarSurface> hierarchy_planes;
		_object_candidate_extractor->extractObjectCandidates(point_cloud, planar_point_cloud, hierarchy_planes);

		std::cout << "3" << std::endl;


		// extract the clustered planes and objects
		std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clustered_objects;
		std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clustered_planes;
		std::vector<sensor_msgs::PointCloud2> clustered_objects_msgs;
		std::vector<geometry_msgs::PoseStamped> centroids_msgs;
		std::vector<raw_msgs::Object> segmented_objects;

		std::cout << "4" << std::endl;

		unsigned int object_count = 0;
		for (unsigned int i = 0; i < hierarchy_planes.size(); i++) {
			structPlanarSurface plane = hierarchy_planes[i];

			std::cout << "5" << std::endl;

			// save for visualization
			clustered_planes.push_back(plane.pointCloud);

			// process all objects on the plane
			for (unsigned int j = 0; j < plane.clusteredObjects.size(); j++) {
				std::cout << "6" << std::endl;

				pcl::PointCloud<pcl::PointXYZRGBNormal> object = plane.clusteredObjects[j];

				// convert to ROS point cloud for further processing
				sensor_msgs::PointCloud2 cloud;
				pcl::toROSMsg(object, cloud);
				clustered_objects_msgs.push_back(cloud);

				raw_msgs::Object segmented_object;

				// find the image corresponding to the cluster
				if(_extract_obj_in_rgb_img)
				{
					sensor_msgs::ImagePtr img = ExtractRegionOfInterest(cloud, image);
					segmented_object.rgb_image = *img;
					segmented_object.rgb_image.header = input_cloud.header;
				}


				// find the centroid
				geometry_msgs::PoseStamped centroid = ExtractCentroid(object);
				centroids_msgs.push_back(centroid);


				// save all information about the object for publication				
				segmented_object.cluster = cloud;
				segmented_object.pose = centroid;
				segmented_objects.push_back(segmented_object);


				// save for visualization
				clustered_objects.push_back(object);
				++object_count;

				std::cout << "7" << std::endl;
			}
		}

		ROS_INFO("found %d objects on %d planes", object_count, hierarchy_planes.size());


		// remember the result of the segmentation for the service
		_last_segmented_objects.stamp = ros::Time::now();
		_last_segmented_objects.objects = segmented_objects;


		// publish the segmented objects
		raw_msgs::ObjectList object_list;
		object_list.objects = segmented_objects;
		_objects_pub.publish(object_list);


		// publish the point clouds for visualization
		PublishPointClouds(clustered_objects, _objects_points_pub);
		PublishPointClouds(clustered_planes, _surfaces_points_pub);
	} catch (tf::TransformException &ex) {
		ROS_WARN("No tf available: %s", ex.what());
	}


	res = _last_segmented_objects;

	return true;
}


bool ObjectSegmentation::PreparePointCloud(sensor_msgs::PointCloud2 &input, pcl::PointCloud<pcl::PointXYZRGB> &output)
{
	if ((input.width <= 0) || (input.height <= 0) || (input.data.empty())) {
		ROS_INFO_ONCE("[%s] pointCloud Msg empty", _node_name.c_str());
		return false;
	}


	sensor_msgs::PointCloud2 point_cloud_transformed;

	std::string from_frame = input.header.frame_id;
	std::string to_frame = "/base_link";
	if (!_tool_box.transformPointCloud(_tf_listener, from_frame, to_frame, input, point_cloud_transformed)) {
		 ROS_INFO_ONCE("[%s] pointCloud tf transform...failed", _node_name.c_str());
		 return false;
	}
	pcl::fromROSMsg(point_cloud_transformed, output);

	output = _tool_box.filterDistance(output, _dist_min_x, _dist_max_x, "x");
	output = _tool_box.filterDistance(output, _dist_min_y, _dist_max_y, "y");
	output = _tool_box.filterDistance(output, _dist_min_z, _dist_max_z, "z");

	_tool_box.subsampling(output, this->_downsampling_distance);

	if (output.points.empty()) {
		ROS_INFO_ONCE("[%s] point cloud empty after filtering", _node_name.c_str());
		return false;
	}

	return true;
}


geometry_msgs::PoseStamped ObjectSegmentation::ExtractCentroid(const pcl::PointCloud<pcl::PointXYZRGBNormal> &object)
{
	pcl::PointXYZ centroid = _tool_box.pointCloudCentroid(object);

	geometry_msgs::PoseStamped pose;
	pose.header = object.header;
	pose.pose.position.x = centroid.x;
	pose.pose.position.y = centroid.y;
	pose.pose.position.z = centroid.z;
	pose.pose.orientation.x = 0.0;
	pose.pose.orientation.y = 0.0;
	pose.pose.orientation.z = 0.0;
	pose.pose.orientation.w = 1.0;

	return pose;
}


IplImage *ObjectSegmentation::ClusterToImage(const sensor_msgs::PointCloud2 &cluster)
{
	pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
	pcl::fromROSMsg(cluster, pcl_cloud);

	sensor_msgs::ImagePtr received_image(new sensor_msgs::Image());
	pcl::toROSMsg(pcl_cloud, *received_image);

	return _bridge.imgMsgToCv(received_image, "rgb8");
}


sensor_msgs::ImagePtr ObjectSegmentation::ExtractRegionOfInterest(const sensor_msgs::PointCloud2 &cluster, IplImage *image)
{
	RegionOfInterest roi = _roi_extractor->Extract(cluster);

	// make sure that only valid pixels are indexed in the image
	if (roi.x_offset >= (unsigned int)image->width) roi.x_offset = image->width - 1;
	if (roi.y_offset >= (unsigned int)image->height) roi.y_offset = image->height - 1;
	if ((roi.x_offset + roi.width) >= image->width) roi.width = image->width - roi.x_offset - 1;
	if ((roi.y_offset + roi.height) >= image->height) roi.height = image->height - roi.y_offset - 1;

	if ((roi.width == 0) || (roi.height == 0)) {
		return sensor_msgs::ImagePtr(new sensor_msgs::Image());
	}

	ROS_DEBUG("ROI: left=%u - top=%u - width=%u - height=%u", roi.x_offset, roi.y_offset, roi.width, roi.height);

	cvSetImageROI(image, cvRect(roi.x_offset, roi.y_offset, roi.width, roi.height));

	// destination image (cvGetSize returns the width and the height of ROI)
	IplImage *sub_image = cvCreateImage(cvGetSize(image),
			image->depth,
			image->nChannels);

	// copy subimage
	cvCopy(image, sub_image, NULL);

	// reset the ROI so that the complete image can be accessed again
	cvResetImageROI(image);

	return _bridge.cvToImgMsg(sub_image, "rgb8");
}


void ObjectSegmentation::PublishPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > &cloud, ros::Publisher &publisher)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud_rgb;
	_tool_box.markClusteredPointCloud(cloud, point_cloud_rgb);

	sensor_msgs::PointCloud2 cloud_rgb;
	pcl::toROSMsg(point_cloud_rgb, cloud_rgb);

	publisher.publish(cloud_rgb);
}
