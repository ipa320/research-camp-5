#include "object_segmentation.h"


int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_segmentation");

	ObjectSegmentation segmentation("object_segmentation");

	ros::spin();

	return 0;
}
