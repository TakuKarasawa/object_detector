#include "point_cloud_object_detector/point_cloud_object_detector.h"

PointCloudObjectDetector::PointCloudObjectDetector() : private_nh_("~")
{
    pc_sub_ = nh_.subscribe("/camera/depth_registered/points",1,&PointCloudObjectDetector::sensor_callback,this);
    bbox_sub_ = nh_.subscribe("/darknet_ros/bounding_boxes",1,&PointCloudObjectDetector::bbox_callback,this);
    obj_pub_ = nh_.advertise<object_detector_msgs::ObjectPositions>("/object_positions",1);
}

void PointCloudObjectDetector::sensor_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    has_received_pcl2 = true;
    cloud->points.clear();
    pcl::fromROSMsg(*msg,*cloud);
}

void PointCloudObjectDetector::bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
    has_received_bbox = true;
    bboxes = *msg;
}

void PointCloudObjectDetector::calc_object_position()
{
    object_detector_msgs::ObjectPositions positions;
    for(const auto &b : bboxes.bounding_boxes){
        std::cout << "Object_Class: " << b.Class << std::endl;
        std::vector<pcl::PointXYZRGB> points;
        std::vector<std::vector<pcl::PointXYZRGB>> rearranged_points(cloud->height,std::vector<pcl::PointXYZRGB>());
        std::vector<pcl::PointXYZRGB> values;
        object_detector_msgs::ObjectPosition position;

        for(const auto &p : cloud->points) points.push_back(p);

        if(points.size() == cloud->width*cloud->height){
            for(int i = 0; i < cloud->height; i++){
                for(int j = 0; j < cloud->width; j++){
                    rearranged_points.at(i).push_back(points.at(i*cloud->width+j));
                }
            }

            if(!(b.xmin == 0 && b.xmax == 0)){
                for(int x = b.xmin; x < b.xmax; x++){
                    for(int y = b.ymin; y < b.ymax; y++){
                        values.push_back(rearranged_points.at(y).at(x));
                    }
                }

                double sum_x = 0.0;
                double sum_y = 0.0;
                double sum_z = 0.0;
                int finite_count = 0;
                for(const auto &value : values){
                    if(isfinite(value.x) && isfinite(value.y) && isfinite(value.z)){
                        sum_x += value.x;
                        sum_y += value.y;
                        sum_z += value.z;
                        finite_count ++;
                    }
                }

                positions.header.frame_id = "base_link";
                positions.header.stamp = ros::Time::now();
                position.Class = b.Class;
                position.probability = b.probability;
                position.x = sum_x/(double)finite_count;
                position.y = sum_y/(double)finite_count;
                position.z = sum_z/(double)finite_count;

                std::cout << "(X,Y,Z): " << "(" << sum_x << "," << sum_y << "," << sum_z << ")" << std::endl;

                double d = sqrt(pow(sum_x,2)+pow(sum_z,2));
                double theta = atan2(sum_z,sum_x) - M_PI/2;

                std::cout << "distance[m]: : " << d << std::endl;
                std::cout << "theta[rad] : " << theta << std::endl;
                std::cout << std::endl;
            }
        }
        positions.header = bboxes.header;
        positions.object_position.push_back(position);
    }
    obj_pub_.publish(positions);
}

void PointCloudObjectDetector::process()
{
    ros::Rate rate(1);
    while(ros::ok()){
        if(has_received_bbox && has_received_pcl2) calc_object_position();
        ros::spinOnce();
        rate.sleep();
    }
}
