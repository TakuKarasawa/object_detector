#include "point_cloud_object_detector/point_cloud_object_detector.h"

PointCloudObjectDetector::PointCloudObjectDetector() :
    private_nh_("~"),
    cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
    has_received_pc_(false)
{
    private_nh_.param("CAMERA_FRAME_ID",CAMERA_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("IS_CLUSTERING",IS_CLUSTERING_,{true});
    private_nh_.param("IS_PCL_TF",IS_PCL_TF_,{false});
    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("CLUSTER_TOLERANCE",CLUSTER_TOLERANCE_,{0.02});
    private_nh_.param("MIN_CLUSTER_SIZE",MIN_CLUSTER_SIZE_,{100});

    pc_sub_ = nh_.subscribe("pc_in",1,&PointCloudObjectDetector::pc_callback,this);
    bbox_sub_ = nh_.subscribe("bbox_in",1,&PointCloudObjectDetector::bbox_callback,this);
    
    obj_pub_ = nh_.advertise<object_detector_msgs::ObjectPositions>("obj_in",1);
    bbox_pub_ = nh_.advertise<object_detector_msgs::BoundingBox3DArray>("bbox_out",1);

    buffer_.reset(new tf2_ros::Buffer);
    listener_.reset(new tf2_ros::TransformListener(*buffer_));
    broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

void PointCloudObjectDetector::pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    cloud_->clear();
    pcl::fromROSMsg(*msg,*cloud_);
    if(IS_PCL_TF_){
        geometry_msgs::TransformStamped transform_stamped;
        try{
            transform_stamped = buffer_->lookupTransform(CAMERA_FRAME_ID_,msg->header.frame_id,ros::Time(0));
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
        object_detector_msgs::BoundingBox3DArray bboxes_3d;
        for(const auto &bbox : msg->bounding_boxes){
            std::cout << "Object_Class: " << bbox.Class << std::endl;
            std::vector<pcl::PointXYZRGB> points;
            std::vector<std::vector<pcl::PointXYZRGB>> rearranged_points(cloud_->height,std::vector<pcl::PointXYZRGB>());
            std::vector<pcl::PointXYZRGB> values;
            object_detector_msgs::ObjectPosition position;
            object_detector_msgs::BoundingBox3D bbox_3d;

            for(const auto &p : cloud_->points) points.push_back(p);

            if(points.size() == cloud_->width*cloud_->height){
                for(int i = 0; i < cloud_->height; i++){
                    for(int j = 0; j < cloud_->width; j++){
                        rearranged_points.at(i).push_back(points.at(i*cloud_->width+j));
                    }
                }

                if(!(bbox.xmin == 0 && bbox.xmax == 0)){
                    for(int x = bbox.xmin; x < bbox.xmax; x++){
                        for(int y = bbox.ymin; y < bbox.ymax; y++){
                            values.push_back(rearranged_points.at(y).at(x));
                        }
                    }

                    double sum_x = 0.0;
                    double sum_y = 0.0;
                    double sum_z = 0.0;
                    int finite_count = 0;

                    double x_max, x_min;
                    double y_max, y_min;
                    double z_max, z_min;
                    if(IS_CLUSTERING_){
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rearranged_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
                        rearranged_cloud->width = bbox.xmax - bbox.xmin;
                        rearranged_cloud->height = bbox.ymax - bbox.ymin;
                        rearranged_cloud->points.resize(rearranged_cloud->width*rearranged_cloud->height);

                        int c = 0;
                        for(const auto &value : values){
                            if(!std::isnan(value.x) && !std::isnan(value.y) && !std::isnan(value.z)){
                                rearranged_cloud->points.at(c).x = value.x;
                                rearranged_cloud->points.at(c).y = value.y;
                                rearranged_cloud->points.at(c).z = value.z;
                                rearranged_cloud->points.at(c).r = value.r;
                                rearranged_cloud->points.at(c).g = value.g;
                                rearranged_cloud->points.at(c).b = value.b;
                                c ++;
                            }
                        }

                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr clustered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                        clustering(rearranged_cloud,clustered_cloud);

                        for(const auto &p : clustered_cloud->points){
                            if(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)){
                                sum_x += p.x;
                                sum_y += p.y;
                                sum_z += p.z;
                                finite_count ++;
                            }
                        }
                    }
                    else{
                        x_max, x_min = values.at(0).x;
                        y_max, y_min = values.at(0).y;
                        z_max, z_min = values.at(0).z;
                        for(const auto &value : values){
                            if(std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z)){
                                sum_x += value.x;
                                sum_y += value.y;
                                sum_z += value.z;

                                if(x_max < value.x) x_max = value.x;
                                if(x_min > value.x) x_min = value.x;
                                if(y_max < value.y) y_max = value.y;
                                if(y_min > value.y) y_min = value.y;
                                if(z_max < value.z) z_max = value.z;
                                if(z_min > value.z) z_min = value.z;
                                finite_count ++;
                            }
                        }
                    }

                    // object_positions
                    positions.header.frame_id = CAMERA_FRAME_ID_;
                    positions.header.stamp = ros::Time::now();
                    position.Class = bbox.Class;
                    position.probability = bbox.probability;
                    position.x = sum_x/(double)finite_count;
                    position.y = sum_y/(double)finite_count;
                    position.z = sum_z/(double)finite_count;

                    double d = std::sqrt(std::pow(position.x,2) + std::pow(position.z,2));
                    double theta = std::atan2(position.z,position.x) - M_PI/2;

                    std::cout << "(X,Y,Z): " << "(" << position.x << "," << position.y << "," << position.z << ")" << std::endl;
                    std::cout << "distance[m]: : " << d << std::endl;
                    std::cout << "theta[rad] : " << theta << std::endl;
                    std::cout << std::endl;

                    bbox_3d.name = bbox.Class;
                    bbox_3d.probability = bbox.probability;
                    bbox_3d.x = position.x;
                    bbox_3d.x_max = x_max;
                    bbox_3d.x_min = x_min;
                    bbox_3d.y = position.y;
                    bbox_3d.y_max = y_max;
                    bbox_3d.y_min = y_min;
                    bbox_3d.z = position.z;
                    bbox_3d.z_max = z_max;
                    bbox_3d.z_min = z_min;
                }
            }
            positions.object_position.push_back(position);
            bboxes_3d.boxes.emplace_back(bbox_3d);
        }
        obj_pub_.publish(positions);
        bbox_pub_.publish(bboxes_3d);
    }
}

void PointCloudObjectDetector::clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,
                                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
{
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(input_cloud);

    std::vector<pcl::PointIndices> indices;
    pcl::shared_ptr<pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>> ec(new pcl::EuclideanClusterExtraction<pcl::PointXYZRGB>);
    ec->setClusterTolerance(CLUSTER_TOLERANCE_);
    ec->setMinClusterSize(MIN_CLUSTER_SIZE_);
    ec->setMaxClusterSize(input_cloud->points.size());
    ec->setSearchMethod(tree);
    ec->setInputCloud(input_cloud);
    ec->extract(indices);

    pcl::shared_ptr<pcl::ExtractIndices<pcl::PointXYZRGB>> ex(new pcl::ExtractIndices<pcl::PointXYZRGB>);
    ex->setInputCloud(input_cloud);
    ex->setNegative(false);

    pcl::PointIndices::Ptr tmp_clustered_indices (new pcl::PointIndices);
    if(indices.size() <= 1) return;
    *tmp_clustered_indices = indices[0];
    ex->setIndices(tmp_clustered_indices);
    ex->filter(*output_cloud);
}

void PointCloudObjectDetector::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}