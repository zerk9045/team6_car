Spring Quarter 2024 EEC 195AB Team 6 - Raphael, Gabriel, and Adrien

This repo serves as the location for all the code needed to run our EEC 195AB Senior Design Project, which was the Autonomous Vehicle Design Project. Our specific project deviates from the default line following/lane following project offered in EEC 195AB. We chose to design a SLAM-capable autonomous navigation vehicle.

TLDR: The goal of this project was to create a map of a floor in Kemper Hall at UC Davis using SLAM and then use the created map to navigate autonomously using waypoints. A video of our project in action can be found [here](https://video.ucdavis.edu/media/EEC+195AB+-+Team6+SLAM+and+Autonomous+Navigation+Vehicle+Design/1_n3ral022). There is also a report of this project which can be found [here](https://drive.google.com/file/d/1YjDNlJzggsWzGPSiC08AWJQSZsNJ5PF0/view?usp=sharing).

As we as a team feel this project has not reached its full potential, Adrien will be periodically updating the vehicle on his own time as a side project. Future plans are to address the oscillatory nature of the car when navigating autonomously, as well as address the networking issue between the remote machine and the Pi 4B.

For the ros2-control custom hardware interface from the Pi 4B to Pico we created, the link to the repo can be found [here](https://github.com/zerk9045/real_car).

A summary of this design project follows.

This project used a Traxxas Rustler RC car as a base platform. All the stock electronics were ripped out, leaving only the servo and brushed DC motors equipped on the car. To replace the stock electronics, we designed a custom motor controller PCB that utilizes a VNH5019A-E H-bridge IC to interface with the onboard motors and a Raspberry Pi Pico microcontroller for control feedback. Due to the Traxxas Rustler not having a motor encoder, we implemented a basic TCRT5000 IR sensor mounted on the brushed DC motor as a speed sensor, which is directly wired to the Pico.

To send control signals and process odometry data, we have a Raspberry Pi 4B running ROS 2. Depending on the configuration, either a remote machine (such as a laptop) or the onboard Pi 4B can run the SLAM and navigation algorithms. The Pi 4B and Pico are connected via standard USB. However, we are running microROS on the Pico for better integration into the ROS 2 node/topic framework we developed. Running microROS on the Pico allows the Pico to be seen by ROS 2 as a single node or multiple nodes for controlling the motors, effectively leaving the Pi 4B to focus solely on computation for SLAM and/or navigation. While a direct UART connection can be established between the Pi 4B and Pico, running microROS frees the Pi 4B from processing and interpreting the raw data coming from the Pico and instead sees standard ROS 2 messages which can be either processed easier or offloaded to other nodes running on the remote machine. The same can be said for data coming from the remote machine and sent to the Pico.

For SLAM, we integrated the FHL-LD19 LiDAR sensor into our design. We chose this LiDAR sensor in particular for its affordability and ROS 2 compatibility. The standard EEC 195AB lane-following project does not make use of other sensors besides ultrasonic sensors, Pi cameras, and a Bluetooth module. Teams who proposed a different project were allocated a budget of $75 for any extra hardware needed. The FHL-LD19 was the only sensor our project required and was within our budget, making it the perfect choice. The FHL-LD19 is connected to the Pi 4B via standard USB, with the necessary ROS 2 node being available for download from the manufacturer’s website.

Our goal for the hardware side of this project was to integrate all the necessary functionality onto the car without interrupting the stock aesthetic of the Traxxas Rustler. We accomplished this with minimal modification to the shell and by 3D printing custom parts. To integrate the FHL-D19, we simply cut out a portion of the top shell and simply mounted the FHL-D19 using screws. To house all the electronics, we 3D printed a custom base plate and level plates to create a seamless hardware stack underneath the top shell. The custom base plate uses existing holes in the car where the stock electronics used to sit. To separate the different layers of electronics, nylon spacers were used. Altogether, our vehicle looks exactly like a stock Traxxas Rustler with a LiDAR sensor mounted to the top.

Overall, this project proved to be very difficult yet very exciting and rewarding. While our project idea was very ambitious for a two-month timeline, we were able to persevere and reach our goal. For some context, the first part of this course, EEC 195A, was conducted from January to the end of March 2024. During this time, we learned all the requisite knowledge to complete the default lane-following project. This included learning the basics of an H-bridge motor controller, basic PCB design and development using Altium Designer, basic PID control theory, and some computer vision techniques/theory using OpenCV. The second part of the course, EEC 195B was conducted from the beginning of April to the end of May 2024. This part of the course was when all the teams applied their knowledge from EEC 195A and implemented it into a working physical design. However, for our team who proposed an alternative project, we essentially started from the backfoot as we needed to do extensive research on ROS 2, SLAM, and autonomous navigation which were not taught during EEC 195A. So to be able to have completed our project within two months while teaching ourselves new concepts along the way was a testament to our perseverance and dedication to completing a successful complex engineering design project.





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
