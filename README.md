# BFMC - Brain ROS Project - FMI RoadRunner

This GitHub repository contains a fork of the [BFCM 2022](https://github.com/ECC-BFMC/Brain_ROS) software with pre-given packages and examples demonstrating how to communicate with them. The pre-installed SD card is already configured to communicate with demo scripts within the ECC-BFCM/Computer project and these scripts are set to run at start-up. However, if you wish to remove this configuration for development purposes, simply clean out the /etc/rc.local file. In addition to the main version, the repository also includes custom ROS nodes for line detection, semaphore, person detection, and traffic sign detection.


## Resources

### ROS Tutorials

* [ROS Wiki](http://wiki.ros.org/)
* [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
	* [Creating Packages and Nodes](https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/Creating-a-ROS-Package-and-Node.html)
	* [Writing a Simple Publisher and Subscriber (Python)](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
	* [Writing Publisher/Subscriber with Parameters, Dynamic Reconfigure and Custom Messages (Python)](http://wiki.ros.org/ROSNodeTutorialPython)

* [ROS Answers](https://answers.ros.org/)

### Libraries and Tools

* [roscpp](http://wiki.ros.org/roscpp) - C++ implementation of the ROS client library.
* [rospy](http://wiki.ros.org/rospy) - Python implementation of the ROS client library.
* [catkin](http://wiki.ros.org/catkin) - Build system for ROS.
* [rosbag](http://wiki.ros.org/rosbag) - Tool for recording and playing back ROS messages.

