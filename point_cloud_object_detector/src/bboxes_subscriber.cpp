#include <ros/ros.h>

#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"

class BboxesSubscriber
{
public:
    BboxesSubscriber() { bboxes_sub_ = nh_.subscribe("darknet_ros/bounding_boxes",1,&BboxesSubscriber::bboxes_callback,this); }
    void process() { ros::spin(); }


private:
    void bboxes_callback(const darknet_ros_msgs::BoundingBoxesConstPtr& msg)
    {
        ROS_INFO("has received bboxes");
        for(size_t i = 0; i < msg->bounding_boxes.size(); i++){
            std::cout << "Class: " << msg->bounding_boxes[i].Class << std::endl;
            std::cout << "Id: " << msg->bounding_boxes[i].id << std::endl;
            std::cout << "Probability: " << msg->bounding_boxes[i].probability << std::endl;
            std::cout << "X_range: " << "[" << msg->bounding_boxes[i].xmin << "," << msg->bounding_boxes[i].xmax << std::endl;
            std::cout << "Y_range: " << "[" << msg->bounding_boxes[i].ymin << "," << msg->bounding_boxes[i].ymax << std::endl;
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber bboxes_sub_;
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"bboxes_subscriber");
    BboxesSubscriber bboxes_subscriber;
    bboxes_subscriber.process();
    return 0;
}
