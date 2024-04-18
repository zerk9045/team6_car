EEC 195AB Team 6 - Raphael, Gabriel, and Adrien

source /opt/ros/humble/setup.bash # do this for every terminal you open
cd dev_ws/src
colcon build --symlink-install    # do this to everytime you update a file
source install/setup.bash         # do this for every terminal you opem

ros2 launch team6_car launch_sim.launch.py    # this will launch gazebo and everything
