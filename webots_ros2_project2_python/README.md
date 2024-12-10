# Project Part 3 ROS2

This package includes the controller and launch file for Part 3 of the Project in CS 460/560, the analysis of coverage. The controller utilizes a wall following method to complete its coverage. The robot uses its right lidar readings to maneuver the room and determine if it is too close or far from the wall and will adjust accordingly. 

#### Wall Following Method
To begin, the robot checks if the right lidar is measuring less than its safe stop distance of 0.25 (the addition of a stop distance of 0.2, to ensure the robot would have time to stop, and a lidar error factor of 0.05, to account for any error in the lidar), which would indicate that the robot is too close to the right wall. If this is true, the robot will turn slightly to the left 0.1 at a rate of 0.10, to accommodate for the closeness. If the right lidar is reading greater than the safe stop distance plus 0.2, making a 0.45 minimum reading, the robot realizes that it is too far from the right wall and it turns right at -0.18 at a rate of 0.1. Otherwise, the robot feels that the distance to the wall is optimal, and it will move forward at a rate of 0.22.

#### Obstacle Avoidance
In order to properly handle objects in its path, the robot recognizes when the front lidar has a reading less than the lidar avoid distance of 0.7. The robot slows down to a rate of 0.07 and turns in order to avoid the object. The direction it turns depends on how long it has been since it has seen a wall. When the robot follows the wall at a too close or optimal length, it sets a found_wall flag as True and updates the time_last_wall variable with the current time. If the right lidar is at least 1.05 and the time since it last found the wall is greater than 5 seconds, then it sets the flag as false. This flag is used when the robot encounters an obstacle in front of it, as if the right lidar is greater than 1, but it has recently seen the wall, it will turn right, otherwise, it will turn left. This logic is due to wanting the robot to turn left in order to find the wall on its right side if it is unable to rediscover it, but if there is a situation in which the wall turns right, such as a door opening, then I want the robot to continue to follow the wall through the door opening and turn right.


### TO INSTALL PACKAGE FOR ASSIGNMENT - From monicadelaine/f24_robotics Github 

1. Set up environment variables for ROS. Make sure to replace '/home/rpi/shared' with your own shared folder location
<pre>
source /opt/ros/humble/setup.bash
</pre>
Also do any Windows or Mac specific setup

For example in Mac...
<pre>
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py
</pre>

For example in windows...
<pre>
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
</pre>

2. Fork your own repository of f23_robotics (using web interface)

3. Clone your fork
<pre>
git clone <your github url for this repository>
</pre>

4. Make the package (for python, it really just installs the files
<pre>
cd f24_robotics
colcon build
</pre>

5. Set up variables to use the package you just created
<pre>
source install/setup.bash
</pre>

6. Start webots simulation with connect back to ROS in the virtual machine
<pre>
ros2 launch webots_ros2_project2_python f23_robotics_1_launch.py
</pre>


### TEST THE CONNECTION BETWEEN ROS2 AND WEBOTS

Test the connection between webots and ROS, use a ROS based ASCII keyboard to move the simulated robot in Webots

1. Open another terminal

2. Redo the source commands (you can add to your bash to execute it automatically each time) 
<pre>
source /opt/ros/humble/setup.bash
source install/setup.bash
</pre>

3. Run the ROS-based keyboard
<pre>
ros2 run teleop_twist_keyboard teleop_twist_keyboard
</pre>


### TO VISUALIZE LASER DATA

1. Open another terminal

2. Redo the source commands (you can add to your bash to execute it automatically each time) 
<pre>
source /opt/ros/humble/setup.bash
source install/setup.bash
</pre>

3. Run the ROS-based keyboard
<pre>
ros2 run teleop_twist_keyboard teleop_twist_keyboard
</pre>
<pre>
  rviz2
</pre>

### RUN SAMPLE CONTROLLER

<pre>
ros2 run webots_ros2_project2_python webots_ros2_project2_python
</pre>

