// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <raw_object_finder/object_segmentationConfig.h>

// ROS message includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <brics_3d_msgs/GetSceneObjects.h>



#include <object_segmentation_common.cpp>


class object_segmentation_ros
{
	public:
		ros::NodeHandle n_;
		
		dynamic_reconfigure::Server<raw_object_finder::object_segmentationConfig> server;
  		dynamic_reconfigure::Server<raw_object_finder::object_segmentationConfig>::CallbackType f;
		

		ros::Publisher object_points_;
		ros::Publisher plane_points_;
		

	ros::ServiceServer get_scene_objects_;
        
 
        object_segmentation_data component_data_;
        object_segmentation_config component_config_;
        object_segmentation_impl component_implementation_;

        object_segmentation_ros()
        {
       	
  			f = boost::bind(&object_segmentation_ros::configure_callback, this, _1, _2);
  			server.setCallback(f);
        	
        	
        		std::string get_scene_objects_remap;
        		n_.param("get_scene_objects_remap", get_scene_objects_remap, (std::string)"get_scene_objects");
        		get_scene_objects_ = n_.advertiseService<brics_3d_msgs::GetSceneObjects::Request , brics_3d_msgs::GetSceneObjects::Response>(get_scene_objects_remap, boost::bind(&object_segmentation_impl::callback_get_scene_objects, &component_implementation_,_1,_2,component_config_));
        
				object_points_ = 	n_.advertise<sensor_msgs::PointCloud2>("object_points", 1);
				plane_points_ = 	n_.advertise<sensor_msgs::PointCloud2>("plane_points", 1);
  	

				n_.param("camera_frame", component_config_.camera_frame, (std::string)"/openni_rgb_optical_frame");
				n_.param("extract_obj_in_rgb_img", component_config_.extract_obj_in_rgb_img, (bool)false);
				n_.param("min_x", component_config_.min_x, (double)0.25);
				n_.param("max_x", component_config_.max_x, (double)1.5);
				n_.param("min_y", component_config_.min_y, (double)-1.0);
				n_.param("max_y", component_config_.max_y, (double)1.0);
				n_.param("min_z", component_config_.min_z, (double)-0.1);
				n_.param("max_z", component_config_.max_z, (double)1.0);
				n_.param("threshold_points_above_lower_plane", component_config_.threshold_points_above_lower_plane, (double)0.01);
				n_.param("downsampling_distance", component_config_.downsampling_distance, (double)0.005);
				n_.param("min_points_per_objects", component_config_.min_points_per_objects, (int)11);
				n_.param("spherical_distance", component_config_.spherical_distance, (double)2.5);
				n_.param("point_cloud_in", component_config_.point_cloud_in, (std::string)"/camera/rgb/points");
				n_.param("min_planar_area_size", component_config_.min_planar_area_size, (double)0.005);
            
        }
		
        
		
		void configure_callback(raw_object_finder::object_segmentationConfig &config, uint32_t level) 
		{
				component_config_.camera_frame = config.camera_frame;
				component_config_.extract_obj_in_rgb_img = config.extract_obj_in_rgb_img;
				component_config_.min_x = config.min_x;
				component_config_.max_x = config.max_x;
				component_config_.min_y = config.min_y;
				component_config_.max_y = config.max_y;
				component_config_.min_z = config.min_z;
				component_config_.max_z = config.max_z;
				component_config_.threshold_points_above_lower_plane = config.threshold_points_above_lower_plane;
				component_config_.downsampling_distance = config.downsampling_distance;
				component_config_.min_points_per_objects = config.min_points_per_objects;
				component_config_.spherical_distance = config.spherical_distance;
				component_config_.point_cloud_in = config.point_cloud_in;
				component_config_.min_planar_area_size = config.min_planar_area_size;
		}

        void configure()
        {
			component_implementation_.configure(component_config_);
        }

        void update()
        {
            component_implementation_.update(component_data_, component_config_);
				object_points_.publish(component_data_.out_object_points);
				plane_points_.publish(component_data_.out_plane_points);
    
        }
 
};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "object_segmentation");

	object_segmentation_ros node;
    node.configure();

	
 	ros::Rate loop_rate(5.0); // Hz

	while(node.n_.ok())
	{
        node.update();
		loop_rate.sleep();
		ros::spinOnce();
	}
	
    return 0;
}
