#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <pcl/point_types_conversion.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <std_msgs/Header.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <string>
#include <vector>

#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"

class TestMessageFilters
{
public:
	TestMessageFilters();
	~TestMessageFilters();
	void process();
private:
	void callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg,const darknet_ros_msgs::BoundingBoxesConstPtr& bbox_msg);

	ros::NodeHandle nh_;
	ros::NodeHandle private_nh_;

	std::string pc_topic_name;
	std::string bbox_topic_name;


	message_filters::Subscriber<sensor_msgs::PointCloud2> *pc_sub_;
	message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> *bbox_sub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,darknet_ros_msgs::BoundingBoxes> SYNCPOLICY;
	message_filters::Synchronizer<SYNCPOLICY> *syncpolicy;
};

TestMessageFilters::TestMessageFilters() : private_nh_("~")
{
	private_nh_.param("pc_topic_name",pc_topic_name,{"/camera/depth_registered/points"});
	private_nh_.param("bbox_topic_name",bbox_topic_name,{"/darknet_ros/bounding_boxes"});

	pc_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_,pc_topic_name,1);
	bbox_sub_ = new message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes>(nh_,bbox_topic_name,1);
	syncpolicy = new message_filters::Synchronizer<SYNCPOLICY>(SYNCPOLICY(10),*pc_sub_,*bbox_sub_);
	syncpolicy->registerCallback(boost::bind(&TestMessageFilters::callback,this,_1,_2));
}

TestMessageFilters::~TestMessageFilters()
{
	delete this->pc_sub_;
	delete this->bbox_sub_;
	delete this->syncpolicy;
}

void TestMessageFilters::callback(const sensor_msgs::PointCloud2ConstPtr& pc_msg,const darknet_ros_msgs::BoundingBoxesConstPtr& bbox_msg)
{
	ROS_INFO("has received!");
	for(size_t i = 0; i < bbox_msg->bounding_boxes.size(); i++){
		std::cout << "Class: " << bbox_msg->bounding_boxes[i].Class << std::endl;
            	std::cout << "Id: " << bbox_msg->bounding_boxes[i].id << std::endl;
            	std::cout << "Probability: " << bbox_msg->bounding_boxes[i].probability << std::endl;
            	std::cout << "X_range: " << "[" << bbox_msg->bounding_boxes[i].xmin << "," << bbox_msg->bounding_boxes[i].xmax << "]"  << std::endl;
           	std::cout << "Y_range: " << "[" << bbox_msg->bounding_boxes[i].ymin << "," << bbox_msg->bounding_boxes[i].ymax << "]"  << std::endl;
	}
}

void TestMessageFilters::process() { ros::spin(); }

int main(int argc,char** argv)
{
	ros::init(argc,argv,"test_message_filters");
	TestMessageFilters test;
	test.process();
	return 0;
}