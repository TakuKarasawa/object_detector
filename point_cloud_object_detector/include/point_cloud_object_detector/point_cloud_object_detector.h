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
#include <algorithm>

#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "object_detector_msgs/ObjectPosition.h"

class PointCloudObjectDetector {
public:
    PointCloudObjectDetector(ros::NodeHandle nh = ros::NodeHandle(),ros::NodeHandle private_nh = ros::NodeHandle("~"));
    void process();

private:
    void sensor_callback(const sensor_msgs::PointCloud2ConstPtr& msg);
    void bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);

    void check_bbox(darknet_ros_msgs::BoundingBox bbox);
    void bbox_process();
    void bbox_process_2();

    bool has_received_bbox = false;
    bool has_received_pcl2 = false;
    
    darknet_ros_msgs::BoundingBoxes bboxes;
    darknet_ros_msgs::BoundingBoxes pre_bboxes;
    //sensor_msgs::PointCloud2 pc;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZRGB>};

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber pc_sub_;
    ros::Subscriber bbox_sub_;
    ros::Publisher pos_pub_;
};

#endif  // POINT_CLOUD_OBJECT_DETECTOR_H_
