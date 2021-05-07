# comp0129rsmi

Code written by Alejandro Halpern Pastor, Andras Nagy, Xingjian Lu and Tim Andersson. 
*LICENSE available in LICENSE.txt

### Description
This code package allows the execution of multiple commands described by the coursework brief on the robot manipulator upon launching the launch file and upon certain keypresses (also described by the coursework brief).

For further details on how the methods work please refer to the thoroughly commented code files cw3.cpp, cw3.h, test_cw3.cpp, and cylinder_segment.cpp.

### Running Instruction:
In order to run our code, this entire package must be placed in the src folder inside a catkin workspace. Then directly under the workspace run the following in terminal:
```
catkin clean
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
roslaunch cw3 cw3.launch
```
All questions are to be run as described in the coursework brief, with no alterations. 

### Reference
for q1:
https://pointclouds.org/documentation/tutorials/voxel_grid.html
https://pointclouds.org/documentation/tutorials/statistical_outlier.html
https://pointclouds.org/documentation/tutorials/passthrough.html
https://pointclouds.org/

For q2 and q3:
Code was largely drawn from ROS moveit tutorials:
http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/pick_place/pick_place_tutorial.html

ROS documentation:
http://wiki.ros.org

<!--
For your code you will required to provide ALWAYS a README.txt file, in which you will need to include:
1. A short description of your implementation.
2. Instructions on how to run the code, including any installations that were required.
3. Name of authors/ partners.
4. If any external help (internet, other students, etc) were used.
5. The code license (e.g., the BSD License). This can be either part of the README file or a separate file.
->
