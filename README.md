# object_detector

## Environment
- Ubuntu 20.04
- ROS Noetic
- PCL 1.10
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)

## Topics
### Subscribed topics
- /bounding_boxes (darknet_ros_msgs/BoundingBoxes)
- /camera/depth_registered/points (sensor_msgs/PointCloud2)

### Published topics
- /object_positions ([object_detector_msgs/ObjectPositions](https://github.com/TakuKarasawa/object_detector/blob/master/object_detector_msgs/msg/ObjectPositions.msg))

## How to Use
- start darknet_ros(ex. when you use yolov3)
```
roslaunch darknet_ros yolo_v3.launch
```

- start point_cloud_object_detector_node
```
roslaunch point_cloud_object_detector point_cloud_object_detector.launch
```