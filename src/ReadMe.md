Terminal 1:

source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh

cd ~/testWS

<!-- rm -rf install build log
colcon build
source install/setup.bash -->

export TURTLEBOT3_MODEL=burger
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models

ros2 launch turtlebot3_gazebo empty_world.launch.py


Terminal 2:

source /opt/ros/humble/setup.bash
source ~/testWS/install/setup.bash
ros2 launch braitenberg braitenberg.launch.py

Terminal 3 (Your working dir should be testWS):

ros2 run plotjuggler plotjuggler -l ./graphconfig.xml

