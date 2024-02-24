# BFMC - Brain ROS Project - FMI RoadRunner

This GitHub repository contains a fork of the [BFCM 2022](https://github.com/ECC-BFMC/Brain_ROS) software with pre-given packages and examples demonstrating how to communicate with them. The pre-installed SD card is already configured to communicate with demo scripts within the ECC-BFCM/Computer project, and these scripts are set to run at start-up. However, if you wish to remove this configuration for development purposes, simply clean out the /etc/rc.local file.

_In addition to the main version, the repository also includes custom ROS nodes for line detection, semaphore, person detection, and traffic sign detection added by our team._

## Implementation Details

<div align="center">
    <img src="https://github.com/danadascalescu00/BoschFutureMobilityChallenge/blob/main/images/BFMC2023-FMI_RoadRunner-Architecture.jpg" alt="Architecture">
     <p><b>Architecture<b></p>
</div>


### Perception

The implementation of the perception module is based on two ROS nodes, which can be found in the `./src/perception/src` directory. The first node is responsible for publishing all the detected lanes, while the second node sends a message when a semaphore is detected along with its position. 

#### Lane Detection and Crosswalk Detection
Our code performs lane detection by applying a trapezoidal region of interest and a preprocessing pipeline that includes converting the image to grayscale, an edge detector, applying a Gaussian filter to reduce noise, and using a dilation filter to enhance edges. We then apply the Hough Transform algorithm to detect the lines in the image and calculate the average slope intercept to represent the road markings.

Additionally, our code contains a method called `detect_crosswalk()` that detects the presence of a crosswalk in an input image. This method creates a region of interest in the lower part of the image and detects contours using cv.findContours(). It then filters out small contours and checks if the number of remaining contours is greater than 2, indicating the presence of a crosswalk.

These methods can be used together to provide a more comprehensive understanding of the vehicle's environment. 

#### Object Detection
To detect semaphores and other objects in the environment, we used the Yolo object detector. We were specifically interested in detecting the following object classes: _person, bicycle, car, motorbike, aeroplane, bus, train, truck, traffic light, fire hydrant, stop sign, and parking meter_.

The control module uses the information obtained from lane detection and object detection to make informed decisions about the vehicle's path planning and control.

### Control Implementation Details
The control module utilizes lane and object detection information to make informed decisions about the vehicle's path planning and control. A simple PID algorithm is implemented, with only the Kp parameter set. Additionally, to ensure smooth control of the vehicle's movement, the moving average method is applied to the last five series of lane positions. This approach helps to reduce the effect of sudden changes in lane detection and ensures more stable and predictable control of the vehicle.

## Project stages
[Planning](https://drive.google.com/file/d/1bDNGplvwU5AbuoVqylIRJV2QbeEqFx8x/view?usp=sharing)

1. [Project Status 1](https://www.overleaf.com/read/wjxfqqpjfsmy#f2c250)
2. [Project Status 2](https://www.overleaf.com/read/bfxrmqzkncvx#d7f36a)
3. [Project Status 3](https://www.overleaf.com/read/xtchfkmjwbts#31dc38)
4. [Project Qualifications](https://www.overleaf.com/read/dvdtysypmzvg#0070fe)


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

