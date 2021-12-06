#ifndef POINT_CLOUD_OBJECT_DETECTOR_H_
#define POINT_CLOUD_OBJECT_DETECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_types_conversion.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>

#include <iostream>
#include <fstream>

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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
    //std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clustered_clouds_;

    std::string pc_topic_name;
    std::string bbox_topic_name;
    std::string obj_topic_name;
    std::string obj_frame_name;
    std::string base_link_frame_id_;

    bool has_received_pc_;
    bool is_pcl_tf_;
    bool is_visualize_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> buffer_;
    std::shared_ptr<tf2_ros::TransformListener> listener_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pc_sub_;
    ros::Subscriber bbox_sub_;
    ros::Publisher obj_pub_;
    //std::vector<ros::Publisher> clustered_cloud_pubs_;
};

#endif  // POINT_CLOUD_OBJECT_DETECTOR_H_