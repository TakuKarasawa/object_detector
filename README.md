# object_detector



## Environment
- Ubuntu 20.04
- ROS Noetic
- PCL 1.10
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros)

## Topics
### Subscribed topics
- /bbox_in (darknet_ros_msgs/BoundingBoxes)
- /pc_in (sensor_msgs/PointCloud2)

### Published topics
- /obj_out ([object_detector_msgs/ObjectPositions](https://github.com/TakuKarasawa/object_detector/blob/master/object_detector_msgs/msg/ObjectPositions.msg))

### Published topics (for debug)
- /pc_out (sensor_msgs/PointCloud2)
	- point cloud in bounding boxes
- /cls_pc_out (sensor_msgs/PointCloud2)
	- clustered point cloud in bounding boxes

## Argument
- `is_debug` : publish /pc_out and /cls_pc_put
- `is_clustering` : use a clustering 
- `is_pcl_tf` : use a tf

## Parameters
- `HZ`
	- Main loop rate (default 10 [Hz])
- `CLUSTER TOLERANCE`
	- If the distance between each point is less than `CLUSTER TOLERANCE`, they are in the same group
- `MIN CLUSTER SIZE`
	- Do not publish if the size of target cluster is less than `MIN CLUSTER SIZE`

## How to Use
- start darknet_ros(ex. when you use yolov3)
```
roslaunch darknet_ros yolo_v3.launch
```

- start point_cloud_object_detector_node
```
roslaunch point_cloud_object_detector point_cloud_object_detector.launch
```