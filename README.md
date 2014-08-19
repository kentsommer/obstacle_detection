Obstacle Detection using PCL and ROS
==================

How to use:

* Bring up openni2: `roslaunch openni2_launch openni2.launch --screen`
* Bring up obstacle detection `rosrun obstacle_detection obstacle_detection input:=/camera/depth/points`
  * This will start spamming console with pointcloud before filter and after filter which will let you know if its running correctly
