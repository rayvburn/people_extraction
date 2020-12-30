# people_extraction
ROS package that searches for certain model names at [`/gazebo/model_states`](http://docs.ros.org/en/kinetic/api/gazebo_msgs/html/msg/ModelStates.html) topic and republishes gathered data as [`people_msgs/People.msg`](http://docs.ros.org/en/api/people_msgs/html/msg/People.html) and [`people_msgs/PositionMeasurementArray.msg`](http://docs.ros.org/en/api/people_msgs/html/msg/PositionMeasurementArray.html) to ROS topics with a given names.

Tested under Ubuntu 16.04 and ROS Kinetic.
