#ifndef POINT_CLOUD_OBJECT_DETECTOR_H_
#define POINT_CLOUD_OBJECT_DETECTOR_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>
#include <pcl_ros/transforms.h>

#include <Eigen/Dense>

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
    void clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pc_sub_;
    ros::Subscriber bbox_sub_;
    ros::Publisher obj_pub_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;

    boost::shared_ptr<tf2_ros::Buffer> buffer_;
    boost::shared_ptr<tf2_ros::TransformListener> listener_;
    boost::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

    std::string base_link_frame_id_;
    std::string pc_frame_id_;
    bool is_pcl_tf_;
    bool is_clustering_;
    bool has_received_pc_;

    int HZ_;
    static const int CLUSTER_NUM_ = 3;
    double CLUSTER_TOLERANCE_;
    double MIN_CLUSTER_SIZE_;
};

#endif  // POINT_CLOUD_OBJECT_DETECTOR_H_
