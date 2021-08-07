#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class PCLSubscriber{
public:
	PCLSubscriber() { pcl_sub = nh.subscribe("/camera/depth_registered/points",1,&PCLSubscriber::sensor_callback,this); }
	void process() { ros::spin(); }
private:
	void sensor_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
		ROS_INFO("has received pcl2");
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud {new pcl::PointCloud<pcl::PointXYZRGB>};
		cloud->points.clear();
		pcl::fromROSMsg(*msg,*cloud);

		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
		//pcl::fromROSMsg(*msg,*cloud);
		//sensor_msgs::PointCloud out_pc;
		//sensor_msgs::convertPointCloud2ToPointCloud(*msg,out_pc);
		for(size_t i = 0; i < cloud->points.size(); i++){
			std::cout << "Cloud[" << i << "]: (" 
			          << cloud->points[i].x << ","
				  << cloud->points[i].y << ","
				  << cloud->points[i].z << ")" << std::endl;

			/*
			std::cout << "Cloud[" << i << "]: (" 
			          << out_pc.points[i].x << ","
				  << out_pc.points[i].y << ","
				  << out_pc.points[i].z << ")" << std::endl;
			*/
			
			//if(msg->data[i] != NULL) std::cout << "Data[" << i << "]: " << msg->data[i] << std::endl;
		}
		std::cout << std::endl;
	}

	ros::NodeHandle nh;
	ros::Subscriber pcl_sub;
};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"pcl_subscriber");
	PCLSubscriber pcl_subscriber;
	pcl_subscriber.process();
	return 0;
}