EEC 195AB Team 6 - Raphael, Gabriel, and Adrien



```
# you must run this every time you open a new terminal
source /opt/ros/humble/setup.bash
```
```
# replace dev_ws with whatever workspace you have
cd dev_ws/src
```
```
# the repository in /src if you haven't already

```
```
# this line builds our package. need to run every time a file is changed
colcon build --symlink-install
```
```
# this line is run only once after you first build the package
source install/setup.bash
```
```
# this will launch all the nodes associated with the simulation of the car, including Gazebo and Rviz2
ros2 launch team6_car launch_sim.launch.py
```
