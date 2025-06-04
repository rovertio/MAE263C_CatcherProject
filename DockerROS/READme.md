vcxsrv# Reference for running the system
## Requirements:
This is assuming utilizatin of a Windows system, where the linux GUIs and programs are run through docker and WSL2. Better performance would be possible through a Linux system without using WSL2
- Install Docker Desktop
- Install WSL2
- Install Ubuntu for WSL2 through microsoft store or command line
- Install VcXsrv through the site [Download link](https://vcxsrv.com/)
- Go through the prompted setups for the VcXsrv executor

## Building the container:
- Build the docker image: opening the folder with the dockerfile in Visual Studio Code is best to reduce complications in container naming
- Compose the container (run within the dockerros directory)
```
docker compose up
```
## Building the necessary ROS2 packages:
- Build the necessary ROS2 packages for lunching the simulation:
  - robot_desc : contains the necessary files for model description
  - ball_detector : contains files necessary to simulate camera ball detection
  - controller_node : contains files with controllers implemented for robot actuators
- First source (run within the /home/Catcher directory)
```
source /opt/ros/humble/setup.bash
source install/setup.bash
```
- Build the packages
```
colcon build --packages-select robot_desc ball_detector controller_node
```
## Running the simulation:
This involves both launching the simulation and the associated ROS2 scriipts integrated with it. To replicate the used hardware, two cameras were used to obtain estimates for center and height of the ball in motion. 
- Along with the sourcing for ROS, the following (run within the /home/Catcher directory) is executed to help Ignition locate the source files
```
export IGN_GAZEBO_RESOURCE_PATH=src/robot_desc/install/robot_desc/share/
```
- The simulation environment is initialized with the launch script below
```
ros2 launch robot_desc sim.launch.py
```
- The simulation computer vision is utilized with the launch script below:
  - Implements height and centroid detection. Predicts based on computed velocity
  - Sets threshold for height within detection
```
ros2 launch ball_detector nodeEx.launch.py
```
- The simulation control scripts are initialized with the launch script below: The launch scripts can be adjusted as necessary for selection of scripts and tuning of system
```
ros2 launch controller_node actuation.launch.py
```
- The ball motion is manually initiated by publiishing to an ignition topic with the following command
```
ign topic -t "/world/world_demo/wrench" -m ignition.msgs.EntityWrench -p "entity: {name: 'ping_pong_ball::ping_pong_ball_link', type: LINK}, wrench: {force: {x:-1, z: 8}}"
```
# Reference commands:
- Sourcing ROS2 distro
```
source /opt/ros/humble/setup.bash
```
- Sourcing the install (within the ROS2 workspace directory)
```
source install/setup.bash
```
- Modifying the environmental variable for gazebo resources (add to bash/run in terminal
```
export IGN_GAZEBO_RESOURCE_PATH=src/robot_desc/install/robot_desc/share/
```
- Modifying the environmental variable for message interfaces (troubleshooting)
```
export LD_LIBRARY_PATH=/opt/ros/humble/lib
```
# Refrence media:
