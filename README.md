**Introduction:**
This is an extension of the face detection project by [phil333] (https://github.com/phil333). I have used his face detection project and added a new feature called face overlay where randomly selected images from a set of files in images/ folder are overlayed on the detected face. This was a demo for [Lahore Science Mela] (http://lsm2017.org/) by [CYPHYNETS research lab] (https://cyphynets.lums.edu.pk), LUMS. The aim was to show the significance of science with practical and fun demo.

![image1](/output/face_overlay_pti.jpg?raw=true "Detect face and overlay a batman mask." {width=100px height=100px})

**LAYOUT:**
- face_detection/
  - cfg/:                 dynamic_reconfigure configuration files
  - images/:              contain face images to be overlayed on detected faces
  - include/:             cascade files for the detection
  - launch/:              roslaunch files
  - src/:                 source files
  - CMakeLists.txt:       CMake project configuration file
  - LICENSES:             license agreement
  - package.xml:          ROS/Catkin package file
  - README.txt:           this file

**How to Use this package:**

First run face_tracking launch file:
  - roslaunch face_detection face_tracking.launch
  
Modify the settings so that detected face data is published on /faceCoord topic. The settings of the program can be changed with the ROS rqt_reconfigure setup.
  - rosrun rqt_reconfigure rqt_reconfigure
  
Then run the face_overlay node:
  - rosrun face_detection face_overlay

To see the coordinates published by the nodes, launch the listener node:
  - rosrun face_detection face_listener


Once you have the rqt_reconfigure open, change the input image default
(/camera/image_raw) to your desired input. Additional information about the
different settings are annotated in the dynamic reconfigure setup (hover over
the setting in the rqt_reconfigure for additional information)


**Requirements:**

This node was designed for ROS Indigo and requires a catkin workspace. The node
also makes use of OpenCV. The node has been tested under OpenCV 2.4.8 and 3.0.0.
To be able to get your images into the node, you will need to ROS Vision_OpenCV
package (http://wiki.ros.org/vision_opencv). You will also need a driver which
can read the images from your camera and which can publish these images inside
ROS, like for example ueye_cam (http://wiki.ros.org/ueye_cam) or usb_cam
(http://wiki.ros.org/usb_cam).
