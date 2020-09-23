# cmvision

## cmvision port for ROS Melodic. Tested on Ubuntu 18.04

Steps to get it working:

0.) Clone this repository into your workspace.

1.) This package depends on wxWidgets. Download the source file for wxWidgets from this [link](https://www.wxwidgets.org/downloads/).

2.) Follow these [instructions](https://wiki.wxwidgets.org/Compiling_and_getting_started) to compile and install the package correctly. You can use this [link](https://www.binarytides.com/install-wxwidgets-ubuntu/) as an additional guide but I wouldn't entirely depend on it.

3.) I ran into an issue with the FindwxWidgets.cmake file while compiling the cmvision package with catkin. This file can be located at: /usr/share/cmake-3.10/Modules
I had to use the modified file provided [here](https://github.com/microsoft/vcpkg/issues/4756#issuecomment-620122321). Replace the contents of FindwxWidgets.cmake file with these modifications.

4.) After doing a sudo apt-get update && sudo apt-get upgrade you should be good to go. Now compile cmvision using catkin_make.

5.) In the catkin_ws/src/cmvision/cmvision.launch file, set the camera topic to your camera topic and in the catkin_ws/src/cmvision/colors.txt specify the blob colors you 
need to detect.

6.) After running roslaunch cmvision cmvision.launch, you should be able to see the results visually and by using rostopic echo /blobs
