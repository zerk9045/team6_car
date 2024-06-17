Spring Quarter 2024 EEC 195AB Team 6 - Raphael, Gabriel, and Adrien

This repo serves as the location for all the code needed to run our EEC 195AB Senior Design Project, which was the Autonomous Vehicle Design Project. Our specific project deviates from the default line following/lane following project offered in EEC 195AB. We chose to design a SLAM-capable autonomous navigation vehicle.

The goal of this project was to create a map of a floor in Kemper Hall at UC Davis using SLAM and then use the created map to navigate autonomously using waypoints. A video of our project in action can be found [here](https://video.ucdavis.edu/media/EEC+195AB+-+Team6+SLAM+and+Autonomous+Navigation+Vehicle+Design/1_n3ral022). There is also a report of this project which can be found [here](https://drive.google.com/file/d/1YjDNlJzggsWzGPSiC08AWJQSZsNJ5PF0/view?usp=sharing).

As we as a team feel this project has not reached its full potential, Adrien will be periodically updating the vehicle on his own time as a side project. Future plans are to address the oscillatory nature of the car when navigating autonomously, as well as address the networking issue between the remote machine and the Pi 4B.

For the ros2-control custom hardware interface from the Pi 4B to Pico we created, the link to the repo can be found [here](https://github.com/zerk9045/real_car).

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
