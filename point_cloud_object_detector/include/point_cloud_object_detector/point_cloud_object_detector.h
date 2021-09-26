#ifndef POINT_CLOUD_OBJECT_DETECTOR_H_
#define POINT_CLOUD_OBJECT_DETECTOR_H_

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

#include <string>
#include <vector>

#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "object_detector_msgs/ObjectPosition.h"
#include "object_detector_msgs/ObjectPositions.h"

class PointCloudObjectDetector {
public:
    PointCloudObjectDetector();
    void process();

private:
    void pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZRGB>};

    std::string pc_topic_name;
    std::string bbox_topic_name;
    std::string obj_topic_name;
    std::string obj_frame_name;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pc_sub_;
    ros::Subscriber bbox_sub_;
    ros::Publisher obj_pub_;
};

#endif  // POINT_CLOUD_OBJECT_DETECTOR_H_