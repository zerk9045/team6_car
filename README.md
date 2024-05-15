EEC 195AB Team 6 - Raphael, Gabriel, and Adrien



```
# you must run this every time you open a new terminal
source /opt/ros/humble/setup.bash

# replace robot_ws with whatever workspace you have
cd robot_ws

# this line builds our package and hardware interface. need to run every time a file is changed
colcon build --packages-select real_car team6_car

# this line is run only once after you first build the package
source install/setup.bash

# this will launch all the nodes associated with the car
ros2 launch team6_car launch_robot.launch.py
```

```
# to run SLAM, run this
ros2 launch team6_car online_async_launch.py

# to run localization/AMCL, run this. change map name if necessary. make sure map is in the same directory as to where you are running this command
ros2 launch team6_car localization_launch.py map:=newhallway.yaml

# to run navigation/MPPI/RPP
ros2 launch team6_car navigation_launch.py map_subscribe_transient_local:=true

```
