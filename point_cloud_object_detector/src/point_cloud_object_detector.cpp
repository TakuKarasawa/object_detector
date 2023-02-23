#include "point_cloud_object_detector/point_cloud_object_detector.h"

PointCloudObjectDetector::PointCloudObjectDetector() :
    private_nh_("~"),
    cloud_(new pcl::PointCloud<pcl::PointXYZRGB>),
    pc_frame_id_(std::string("")), has_received_pc_(false)
{
    private_nh_.param("CAMERA_FRAME_ID",CAMERA_FRAME_ID_,{std::string("base_link")});
    private_nh_.param("HZ",HZ_,{10});

    // clustring param
    private_nh_.param("IS_CLUSTERING",IS_CLUSTERING_,{true});
    private_nh_.param("CLUSTER_TOLERANCE",CLUSTER_TOLERANCE_,{0.02});
    private_nh_.param("MIN_CLUSTER_SIZE",MIN_CLUSTER_SIZE_,{100});

    pc_sub_ = nh_.subscribe("pc_in",1,&PointCloudObjectDetector::pc_callback,this);
    bbox_sub_ = nh_.subscribe("bbox_in",1,&PointCloudObjectDetector::bbox_callback,this);
    
    obj_pub_ = nh_.advertise<object_detector_msgs::ObjectPositions>("obj_in",1);
    
    private_nh_.param("IS_DEBUG",IS_DEBUG_,{false});
    if(IS_DEBUG_){
        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pc_out",1);
        if(IS_CLUSTERING_){
            cls_pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cls_pc_out",1);
        }
    }

    private_nh_.param("IS_PCL_TF",IS_PCL_TF_,{false});
    if(IS_PCL_TF_){
        buffer_.reset(new tf2_ros::Buffer);
        listener_.reset(new tf2_ros::TransformListener(*buffer_));
        broadcaster_.reset(new tf2_ros::TransformBroadcaster);
    }
}

void PointCloudObjectDetector::pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    // cloud_->clear();
    pcl::fromROSMsg(*msg,*cloud_);
    pc_frame_id_ = msg->header.frame_id;
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
        ros::Time now_time = ros::Time::now();

        // object positions
        object_detector_msgs::ObjectPositions positions;
        positions.header.frame_id = CAMERA_FRAME_ID_;
        positions.header.stamp = now_time;

        // merged cloud
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_cls_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        for(const auto &bbox : msg->bounding_boxes){
            std::vector<pcl::PointXYZRGB> points;
            for(const auto &p : cloud_->points) points.emplace_back(p);

            std::vector<std::vector<pcl::PointXYZRGB>> rearranged_points(cloud_->height,std::vector<pcl::PointXYZRGB>());
            if(points.size() == cloud_->width*cloud_->height){
                for(int i = 0; i < cloud_->height; i++){
                    for(int j = 0; j < cloud_->width; j++){
                        rearranged_points.at(i).emplace_back(points.at(i*cloud_->width+j));
                    }
                }
            }else{
                ROS_WARN("points size is not cloud size");
                return;
            }

            std::vector<pcl::PointXYZRGB> values;
            if(!(bbox.xmin == 0 && bbox.xmax == 0)){
                for(int x = bbox.xmin; x < bbox.xmax; x++){
                    for(int y = bbox.ymin; y < bbox.ymax; y++){
                        values.emplace_back(rearranged_points.at(y).at(x));
                    }
                }
                
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                obj_cloud->width = bbox.xmax - bbox.xmin;
                obj_cloud->height = bbox.ymax - bbox.ymin;
                obj_cloud->points.resize(obj_cloud->width*obj_cloud->height);
                convert_from_vec_to_pc(values,obj_cloud);
                *merged_cloud += *obj_cloud;

                double x, y, z;              
                if(IS_CLUSTERING_){
                    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cls_obj_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
                    clustering(obj_cloud,cls_obj_cloud);
                    if(cls_obj_cloud->points.empty()) return;
                    calc_position(cls_obj_cloud,x,y,z);
                    *merged_cls_cloud += *cls_obj_cloud;
                }
                else calc_position(obj_cloud,x,y,z);
                
                // object_position
                object_detector_msgs::ObjectPosition position;
                position.Class = bbox.Class;
                position.probability = bbox.probability;
                position.x = x;
                position.y = y;
                position.z = z;
                positions.object_position.emplace_back(position);

                double d = std::sqrt(std::pow(position.x,2) + std::pow(position.z,2));
                double theta = std::atan2(position.z,position.x) - M_PI/2;

                std::cout << "(NAME,X,Y,Z): (" << bbox.Class << "," 
                                               << position.x << "," 
                                               << position.y << "," 
                                               << position.z << ")" << std::endl;
                std::cout << "Distance[m]: : " << d << std::endl;
                std::cout << "Angle[rad] : " << theta << std::endl << std::endl;
            }
            else{
                ROS_WARN("No bbox range");
                return;
            }
        }
        if(positions.object_position.empty()) return;
        obj_pub_.publish(positions);

        if(IS_DEBUG_){
            sensor_msgs::PointCloud2 cloud_msg;
            pcl::toROSMsg(*merged_cloud,cloud_msg);
            cloud_msg.header.frame_id = pc_frame_id_;
            cloud_msg.header.stamp = now_time;
            pc_pub_.publish(cloud_msg);

            if(IS_CLUSTERING_){
                sensor_msgs::PointCloud2 cls_cloud_msg;
                pcl::toROSMsg(*merged_cls_cloud,cls_cloud_msg);
                cls_cloud_msg.header.frame_id = pc_frame_id_;
                cls_cloud_msg.header.stamp = now_time;
                cls_pc_pub_.publish(cls_cloud_msg);
            }
        }
    }
    has_received_pc_ = false;
    cloud_->clear();
}

void PointCloudObjectDetector::convert_from_vec_to_pc(std::vector<pcl::PointXYZRGB>& vec,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc)
{
    int count = 0;
    for(const auto &v : vec){
        if(!std::isnan(v.x) && !std::isnan(v.y) && !std::isnan(v.z)){
            pc->points.at(count).x = v.x;
            pc->points.at(count).y = v.y;
            pc->points.at(count).z = v.z;
            pc->points.at(count).r = v.r;
            pc->points.at(count).g = v.g;
            pc->points.at(count).b = v.b;
            count++;
        }
    }
}

void PointCloudObjectDetector::clustering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& input_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& output_cloud)
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    if(indices.size() <= 1) return;
    *tmp_clustered_indices = indices[0];
    ex->setIndices(tmp_clustered_indices);
    ex->filter(*tmp_cloud);
    output_cloud = tmp_cloud;
}

void PointCloudObjectDetector::calc_position(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,double& x,double& y,double& z)
{
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    int count = 0;
    for(const auto &p : cloud->points){
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
        count++;
    }
    x = sum_x/(double)count;
    y = sum_y/(double)count;
    z = sum_z/(double)count;
}

void PointCloudObjectDetector::process()
{
    ros::Rate rate(HZ_);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}