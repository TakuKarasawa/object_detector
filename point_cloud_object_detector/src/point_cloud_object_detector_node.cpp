#include "point_cloud_object_detector/point_cloud_object_detector.h"

using namespace object_detector;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"point_cloud_object_detector");
    PointCloudObjectDetector point_cloud_object_detector;
    point_cloud_object_detector.process();
    return 0;
}