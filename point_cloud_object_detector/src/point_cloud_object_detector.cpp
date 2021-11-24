#include "point_cloud_object_detector/point_cloud_object_detector.h"

PointCloudObjectDetector::PointCloudObjectDetector() :
    private_nh_("~"), has_received_pc_(false),
    cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    private_nh_.param("pc_topic_name",pc_topic_name,{"/camera/depth_registered/points"});
    private_nh_.param("bbox_topic_name",bbox_topic_name,{"/darknet_ros/bounding_boxes"});
    private_nh_.param("obj_topic_name",obj_topic_name,{"/object_positions"});
    private_nh_.param("obj_frame_name",obj_frame_name,{"base_link"});
    private_nh_.param("base_link_frame_id",base_link_frame_id_,{"base_link"});

    private_nh_.param("is_pcl_tf",is_pcl_tf_,{false});

    pc_sub_ = nh_.subscribe(pc_topic_name,1,&PointCloudObjectDetector::pc_callback,this);
    bbox_sub_ = nh_.subscribe(bbox_topic_name,1,&PointCloudObjectDetector::bbox_callback,this);
    obj_pub_ = nh_.advertise<object_detector_msgs::ObjectPositions>(obj_topic_name,1);

    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));
}

void PointCloudObjectDetector::pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg(*msg,*cloud_);

    if(is_pcl_tf_){
        geometry_msgs::TransformStamped transform_stamped;
        try{
            transform_stamped = buffer_->lookupTransform(base_link_frame_id_,msg->header.frame_id,ros::Time(0));
        }
        catch(tf2::TransformException& ex){
            ROS_WARN("%s", ex.what());
            return;
        }
        Eigen::Matrix4f transform = tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
        pcl::transformPointCloud(*cloud_,*cloud_,transform);
    }
    has_received_pc_ = true;
}

void PointCloudObjectDetector::bbox_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
{
    if(has_received_pc_){
        object_detector_msgs::ObjectPositions positions;
        for(const auto &b : msg->bounding_boxes){
            std::cout << "Object_Class: " << b.Class << std::endl;
            std::vector<pcl::PointXYZRGB> points;
            std::vector<std::vector<pcl::PointXYZRGB>> rearranged_points(cloud_->height,std::vector<pcl::PointXYZRGB>());
            std::vector<pcl::PointXYZRGB> values;
            object_detector_msgs::ObjectPosition position;

            for(const auto &p : cloud_->points) points.push_back(p);

            if(points.size() == cloud_->width*cloud_->height){
                for(int i = 0; i < cloud_->height; i++){
                    for(int j = 0; j < cloud_->width; j++){
                        rearranged_points.at(i).push_back(points.at(i*cloud_->width+j));
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

                    positions.header.frame_id = obj_frame_name;
                    positions.header.stamp = ros::Time::now();
                    position.Class = b.Class;
                    position.probability = b.probability;
                    position.x = sum_x/(double)finite_count;
                    position.y = sum_y/(double)finite_count;
                    position.z = sum_z/(double)finite_count;

                    double d = std::sqrt(std::pow(position.x,2) + std::pow(position.z,2));
                    double theta = std::atan2(position.z,position.x) - M_PI/2;

                    std::cout << "(X,Y,Z): " << "(" << position.x << "," << position.y << "," << position.z << ")" << std::endl;
                    std::cout << "distance[m]: : " << d << std::endl;
                    std::cout << "theta[rad] : " << theta << std::endl;
                    std::cout << std::endl;
                    
                    //static std::ofstream ofs("/home/amsl/catkin_ws/src/object_detector/point_cloud_object_detector/record/record.csv");
                    //ofs << d << std::endl;
                }
            }
            positions.object_position.push_back(position);
        }
        obj_pub_.publish(positions);
    }
    else std::cout << "Don't receive bbox" << std::endl;
}

void PointCloudObjectDetector::process() { ros::spin(); }