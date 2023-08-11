# laserscan_to_pointcloud
A ROS 2 Package to convert Laserscan mesages to PointCloud2 messages. 

This package will convert Laserscan messages from a topic called */scan* to PointCloud2 messages on a topic called */point_cloud*.
To use the package:
1. Clone the repo into the src directory of your ROS 2 workspace.
2. Build the package (*colcon build*).
3. To run the package, do *ros2 run laserscan_to_pointcloud convert*.

The package is currently intended to be for ROS2 Foxy and ROS2 Humble. However, the package may work for other versions of ROS 2, and ROS 1 support may be provided in the future.
