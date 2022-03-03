#include "point_cloud_object_detector/point_cloud_object_detector.h"

PointCloudObjectDetector::PointCloudObjectDetector() : 
    private_nh_("~"), 
    has_received_pc_(false),
    cloud_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
    private_nh_.param("HZ",HZ_,{10});
    private_nh_.param("CLUSTER_TOLERANCE",CLUSTER_TOLERANCE_,{0.02});
    private_nh_.param("MIN_CLUSTER_SIZE",MIN_CLUSTER_SIZE_,{100});
    
    private_nh_.param("base_link_frame_id",base_link_frame_id_,{"base_link"});
    private_nh_.param("is_clustering",is_clustering_,{true});
    private_nh_.param("is_pcl_tf",is_pcl_tf_,{false});

    pc_sub_ = nh_.subscribe("/camera/depth_registered/points",1,&PointCloudObjectDetector::pc_callback,this);
    bbox_sub_ = nh_.subscribe("/bounding_boxes",1,&PointCloudObjectDetector::bbox_callback,this);
    
    obj_pub_ = nh_.advertise<object_detector_msgs::ObjectPositions>("/object_positions",1);

    buffer_.reset(new tf2_ros::Buffer);
	listener_.reset(new tf2_ros::TransformListener(*buffer_));
	broadcaster_.reset(new tf2_ros::TransformBroadcaster);
}

void PointCloudObjectDetector::pc_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    cloud_->clear();
	pcl::fromROSMsg(*msg,*cloud_);
	pc_frame_id_ = msg->header.frame_id;
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
        for(const auto &bbox : msg->bounding_boxes){
            std::cout << "Object_Class: " << bbox.Class << std::endl;
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
                    if(is_clustering_){
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rearranged_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
					    rearranged_cloud->width = bbox.xmax - bbox.xmin;
					    rearranged_cloud->height = bbox.ymax - bbox.ymin;
					    rearranged_cloud->points.resize(rearranged_cloud->width*rearranged_cloud->height);

                        int c = 0;
					    for(const auto &value : values){
						    if(!isnan(value.x) && !isnan(value.y) && !isnan(value.z)){
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
                            if(isfinite(p.x) && isfinite(p.y) && isfinite(p.z)){
                                sum_x += p.x;
                                sum_y += p.y;
                                sum_z += p.z;
                                finite_count ++;
                            }
                        }
                    }
                    else{
                        for(const auto &value : values){
                            if(isfinite(value.x) && isfinite(value.y) && isfinite(value.z)){
                                sum_x += value.x;
                                sum_y += value.y;
                                sum_z += value.z;
                                finite_count ++;
                            }
                        }
                    }

                    positions.header.frame_id = base_link_frame_id_;
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
                }
            }
            positions.object_position.push_back(position);
        }
        obj_pub_.publish(positions);
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