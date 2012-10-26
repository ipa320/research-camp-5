//ROS typedefs
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <raw_srvs/GetObjects.h>
#include <brics_3d_msgs/GetSceneObjects.h>

/* protected region user include files on begin */
#include <raw_msgs/ObjectList.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/CvBridge.h>
#include <std_srvs/Empty.h>
#include <raw_srvs/GetObjects.h>
#include <ros/topic.h>

#include "roi_extraction.h"
#include "toolbox_ros.h"
#include "object_candidate_extraction.h"
#include "struct_planar_surface.h"

#include <brics_3d/worldModel/WorldModel.h>
#include <brics_3d_ros/SceneGraphTypeCasts.h>
/* protected region user include files end */

class object_segmentation_config
{
public:
		std::string camera_frame;
		bool extract_obj_in_rgb_img;
		double min_x;
		double max_x;
		double min_y;
		double max_y;
		double min_z;
		double max_z;
		double threshold_points_above_lower_plane;
		double downsampling_distance;
		int min_points_per_objects;
		double spherical_distance;

};

class object_segmentation_data
{
// autogenerated: don't touch this class
public:
//input data
  
	
//output data
    	sensor_msgs::PointCloud2 out_object_points;
    	sensor_msgs::PointCloud2 out_plane_points;
 

};

class object_segmentation_impl
{
	/* protected region user member variables on begin */
	RoiExtraction *roi_extractor;
	sensor_msgs::CvBridge bridge;
	raw_srvs::GetObjects::Response last_segmented_objects;
	CObjectCandidateExtraction *object_candidate_extractor;
	CToolBoxROS tool_box;
	tf::TransformListener tf_listener;

	object_segmentation_config config;
	object_segmentation_data data;

	brics_3d::WorldModel* wm;
	unsigned int sceneObjectsGroupId;
	brics_3d::rsg::Attribute sceneObjectsTag;
	/* protected region user member variables end */

public:
    object_segmentation_impl() 
    {
        /* protected region user constructor on begin */
		/* protected region user constructor end */
    }
    void configure(object_segmentation_config config) 
    {
        /* protected region user configure on begin */
    	this->config = config;
    	object_candidate_extractor = new CObjectCandidateExtraction(config.threshold_points_above_lower_plane, config.min_points_per_objects, config.spherical_distance);
		roi_extractor = new RoiExtraction(config.camera_frame);

		wm = new brics_3d::WorldModel();
		initializeSceneGraph(wm);
		/* protected region user configure end */
    }
    void update(object_segmentation_data &data, object_segmentation_config config)
    {
        /* protected region user update on begin */
    	data = this->data;
		/* protected region user update end */
    }

	bool callback_get_segmented_objects(raw_srvs::GetObjects::Request  &req, raw_srvs::GetObjects::Response &res , object_segmentation_config config)
	{
		/* protected region user implementation of service callback for get_segmented_objects on begin */
		sensor_msgs::PointCloud2::ConstPtr const_input_cloud;
		sensor_msgs::PointCloud2 input_cloud;
		pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

		do
		{
			const_input_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points", ros::Duration(5));
			ROS_INFO("received point cloud data. Doing preprocessing now ...");
			input_cloud = *const_input_cloud;

		}while(!PreparePointCloud(input_cloud, point_cloud));

		try {
			// start with an empty set of segmented objects
			last_segmented_objects.objects.clear();


			// point cloud colors to color image
			IplImage *image = ClusterToImage(input_cloud);

			// find planes and objects
			pcl::PointCloud<pcl::PointXYZRGBNormal> planar_point_cloud;
			std::vector<structPlanarSurface> hierarchy_planes;

			ROS_INFO("extract object candidates");
			object_candidate_extractor->extractObjectCandidates(point_cloud, planar_point_cloud, hierarchy_planes);

			// extract the clustered planes and objects
			std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clustered_objects;
			std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clustered_planes;
			std::vector<sensor_msgs::PointCloud2> clustered_objects_msgs;
			std::vector<geometry_msgs::PoseStamped> centroids_msgs;
			std::vector<raw_msgs::Object> segmented_objects;

			unsigned int object_count = 0;
			for (unsigned int i = 0; i < hierarchy_planes.size(); i++) {
				structPlanarSurface plane = hierarchy_planes[i];

				// save for visualization
				clustered_planes.push_back(plane.pointCloud);

				// process all objects on the plane
				for (unsigned int j = 0; j < plane.clusteredObjects.size(); j++) {
					pcl::PointCloud<pcl::PointXYZRGBNormal> object = plane.clusteredObjects[j];

					// convert to ROS point cloud for further processing
					sensor_msgs::PointCloud2 cloud;
					pcl::toROSMsg(object, cloud);
					clustered_objects_msgs.push_back(cloud);

					raw_msgs::Object segmented_object;

					// find the image corresponding to the cluster
					if(config.extract_obj_in_rgb_img)
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
				}
			}

			ROS_INFO("found %d objects on %d planes", object_count, hierarchy_planes.size());


			// remember the result of the segmentation for the service
			last_segmented_objects.stamp = ros::Time::now();
			last_segmented_objects.objects = segmented_objects;


			// publish the segmented objects
			raw_msgs::ObjectList object_list;
			object_list.objects = segmented_objects;


			// publish the point clouds for visualization
			PublishPointClouds(clustered_objects, data.out_object_points);
			PublishPointClouds(clustered_planes, data.out_plane_points);
		} catch (tf::TransformException &ex) {
			ROS_WARN("No tf available: %s", ex.what());
		}

		res = last_segmented_objects;

		/* protected region user implementation of service callback for get_segmented_objects end */
		return true;
	}
	bool callback_get_scene_objects(brics_3d_msgs::GetSceneObjects::Request  &req, brics_3d_msgs::GetSceneObjects::Response &res , object_segmentation_config config)
	{
		/* protected region user implementation of service callback for get_scene_objects on begin */
		//this->config = config; //update config (dynamic reconfigure)

		sensor_msgs::PointCloud2::ConstPtr const_input_cloud;
		sensor_msgs::PointCloud2 input_cloud;
		pcl::PointCloud<pcl::PointXYZRGB> point_cloud;

		vector<brics_3d::rsg::Attribute> attributes;
		brics_3d::rsg::TimeStamp currentTime;
		ros::Time time = ros::Time::now();
		brics_3d::rsg::SceneGraphTypeCasts::convertRosMsgToTimeStamp(time, currentTime);

		clearScene(wm, sceneObjectsGroupId);

		do
		{
			const_input_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/rgb/points", ros::Duration(5));
			ROS_INFO("received point cloud data. Doing preprocessing now ...");
			input_cloud = *const_input_cloud;

		}while(!PreparePointCloud(input_cloud, point_cloud));

		try {
			// start with an empty set of segmented objects
			last_segmented_objects.objects.clear();


			// point cloud colors to color image
			IplImage *image = ClusterToImage(input_cloud);

			// find planes and objects
			pcl::PointCloud<pcl::PointXYZRGBNormal> planar_point_cloud;
			std::vector<structPlanarSurface> hierarchy_planes;

			ROS_INFO("extract object candidates");
			object_candidate_extractor->extractObjectCandidates(point_cloud, planar_point_cloud, hierarchy_planes);

			// extract the clustered planes and objects
			std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clustered_objects;
			std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > clustered_planes;
			std::vector<sensor_msgs::PointCloud2> clustered_objects_msgs;
			std::vector<geometry_msgs::PoseStamped> centroids_msgs;
			std::vector<raw_msgs::Object> segmented_objects;

			unsigned int object_count = 0;
			for (unsigned int i = 0; i < hierarchy_planes.size(); i++) {
				structPlanarSurface plane = hierarchy_planes[i];

				// save for visualization
				clustered_planes.push_back(plane.pointCloud);

				// process all objects on the plane
				for (unsigned int j = 0; j < plane.clusteredObjects.size(); j++) {
					pcl::PointCloud<pcl::PointXYZRGBNormal> object = plane.clusteredObjects[j];

					// convert to ROS point cloud for further processing
					sensor_msgs::PointCloud2 cloud;
					pcl::toROSMsg(object, cloud);
					clustered_objects_msgs.push_back(cloud);

					raw_msgs::Object segmented_object;

					// find the image corresponding to the cluster
					if(config.extract_obj_in_rgb_img)
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

					// add found object to scene graph
					unsigned int sceneObjectTransformId;
					brics_3d::HomogeneousMatrix44::IHomogeneousMatrix44Ptr objectPose(new brics_3d::HomogeneousMatrix44());
					brics_3d::rsg::SceneGraphTypeCasts::convertPoseMsgToHomogeniousMatrix(centroid, objectPose);
					attributes.clear();
					attributes.push_back(sceneObjectsTag);
					wm->scene.addTransformNode(this->sceneObjectsGroupId, sceneObjectTransformId, attributes, objectPose, currentTime);

					// save for visualization
					clustered_objects.push_back(object);
					++object_count;
				}
			}

			ROS_INFO("found %d objects on %d planes", object_count, hierarchy_planes.size());


			// remember the result of the segmentation for the service
			last_segmented_objects.stamp = ros::Time::now();
			last_segmented_objects.objects = segmented_objects;


			// publish the segmented objects
			raw_msgs::ObjectList object_list;
			object_list.objects = segmented_objects;


			// publish the point clouds for visualization
			PublishPointClouds(clustered_objects, data.out_object_points);
			PublishPointClouds(clustered_planes, data.out_plane_points);
		} catch (tf::TransformException &ex) {
			ROS_WARN("No tf available: %s", ex.what());
		}



		vector<brics_3d::SceneObject> foundSceneObjects;
		brics_3d::rsg::SceneGraphTypeCasts::convertRosMsgToAttributes(req.attributes, attributes);
		attributes.push_back(sceneObjectsTag);
		wm->getSceneObjects(attributes, foundSceneObjects);
		ROS_INFO("Found %d scene objects", foundSceneObjects.size());
		std::string to_frame = "/base_link";
		brics_3d::rsg::SceneGraphTypeCasts::convertSceneObjectsToRosMsg(foundSceneObjects, res.results, to_frame);

		/* protected region user implementation of service callback for get_scene_objects end */
		return true;
	}
    
    /* protected region user additional functions on begin */
	bool PreparePointCloud(sensor_msgs::PointCloud2 &input, pcl::PointCloud<pcl::PointXYZRGB> &output)
	{
		if ((input.width <= 0) || (input.height <= 0) || (input.data.empty())) {
			ROS_INFO("pointCloud Msg empty");
			return false;
		}


		sensor_msgs::PointCloud2 point_cloud_transformed;

		std::string from_frame = input.header.frame_id;
		std::string to_frame = "/base_link";
		ROS_DEBUG("from_frame: %s, to_frame %s", from_frame.c_str(), to_frame.c_str());
		if (!tool_box.transformPointCloud(tf_listener, from_frame, to_frame, input, point_cloud_transformed)) {
			 ROS_INFO("pointCloud tf transform...failed");
			 return false;
		}
		pcl::fromROSMsg(point_cloud_transformed, output);

		output = tool_box.filterDistance(output, config.min_x, config.max_x, "x");
		output = tool_box.filterDistance(output, config.min_y, config.max_y, "y");
		output = tool_box.filterDistance(output, config.min_z, config.max_z, "z");

		tool_box.subsampling(output, config.downsampling_distance);

		if (output.points.empty()) {
			ROS_INFO("point cloud empty after filtering");
			return false;
		}

		return true;
	}


	geometry_msgs::PoseStamped ExtractCentroid(const pcl::PointCloud<pcl::PointXYZRGBNormal> &object)
	{
		pcl::PointXYZ centroid = tool_box.pointCloudCentroid(object);

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


	IplImage *ClusterToImage(const sensor_msgs::PointCloud2 &cluster)
	{
		pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud;
		pcl::fromROSMsg(cluster, pcl_cloud);

		sensor_msgs::ImagePtr received_image(new sensor_msgs::Image());
		pcl::toROSMsg(pcl_cloud, *received_image);

		return bridge.imgMsgToCv(received_image, "rgb8");
	}


	sensor_msgs::ImagePtr ExtractRegionOfInterest(const sensor_msgs::PointCloud2 &cluster, IplImage *image)
	{
		RegionOfInterest roi = roi_extractor->Extract(cluster);

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

		return bridge.cvToImgMsg(sub_image, "rgb8");
	}


	void PublishPointClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal> > &cloud, sensor_msgs::PointCloud2 &published_data)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal> point_cloud_rgb;
		tool_box.markClusteredPointCloud(cloud, point_cloud_rgb);

		sensor_msgs::PointCloud2 cloud_rgb;
		pcl::toROSMsg(point_cloud_rgb, cloud_rgb);

		published_data = cloud_rgb;
	}

	void initializeSceneGraph(brics_3d::WorldModel *wm) {
		unsigned int groupId;
		vector<brics_3d::rsg::Attribute> attributes;
		sceneObjectsTag = brics_3d::rsg::Attribute("name","scene_object");

		attributes.clear();
		attributes.push_back(brics_3d::rsg::Attribute("name","scene"));
		wm->scene.addGroup(wm->getRootNodeId(), groupId, attributes);
		sceneObjectsGroupId = groupId;
	}

	// Delete all childs of a group node defined by the subGraphId
	void clearScene(brics_3d::WorldModel* wm, unsigned int subGraphId) {
		vector<unsigned int> childIds;
		wm->scene.getGroupChildren(subGraphId, childIds);
		for (unsigned int childId = 0; childId < childIds.size(); ++childId) {
			wm->scene.deleteNode(childIds[childId]);
		}
	}
	/* protected region user additional functions end */
    
};
