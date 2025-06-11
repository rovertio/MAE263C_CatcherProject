vcxsrv# Reference for running the system
## Requirements:
This is assuming utilizatin of a Windows system, where the linux GUIs and programs are run through docker and WSL2. Better performance would be possible through a Linux system without using WSL2
- Install Docker Desktop
- Install WSL2
- Install Ubuntu for WSL2 through microsoft store or command line
- Install VcXsrv through the site [Download link](https://vcxsrv.com/)
- Go through the prompted setups for the VcXsrv executor

##### USING DOCKER FROM PARENT FOLDER #####


## Building the container from MAE263C_CatcherProject Parent Folder

- Build the docker image with a predetermined tag, to reduce complications in image naming:
```
docker build -f DockerROS/Dockerfile -t ubuntu-desktop-ros:humble-gazebo-fortress DockerROS
```

- change the directory to DockerRos, then compose the image:
```
cd DockerROS
docker compose up
```
## Build ROS Package

- Opern a new powershell terminal and open a shell in the runnning container:
```
docker exec -it dockerros-gaz_con-1 bash
```

- Change the directory so you're in the right path:
```
cd /workspaces/ros2_ws
# or
cd /workspaces/simulation_ws ## <- depending on if you're running the simulation or not
```

- Build the packages:
```
colcon build --packages-select robot_desc ball_detector controller_node evaluator_node controll_common controller_msgs pwm_position_node
```

- Source ROS2 and the workspace:
```
source /opt/ros/humble/setup.bash
source install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=src/robot_desc/install/robot_desc/share/
```



## Running the code:

- First you need to source the workspace:
```
source install/setup.bash
```

- Then run whichever node you want:
```
ros2 run [NODE_NAME]
```


##### USING DOCKER FROM DOCKERROS FOLDER #####


## Building the container from separate DockerROS window:
- Build the docker image: opening the folder with the dockerfile in Visual Studio Code is best to reduce complications in container naming
```
docker build .
```
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
  - Computes from two camera images without depth perception
  - Spins the /depth_node and the /pixel_to_xy_bridge nodes
    - /depth_node : sends coordinates within camera frame
      - Implements height and centroid detection. Predicts based on computed velocity
      - Sets threshold for height within detection
    - /pixel_to_xy_bridge : transforms camera coordinates to the task space of the robot
```
ros2 launch ball_detector nodeEx.launch.py
```
- The simulation control scripts are initialized with the launch script below: The launch scripts can be adjusted as necessary for selection of scripts and tuning of system
  - For the decentralized PID node, the gains can be adjusted from the command line via setting ros parameters (the line below the launch)
```
ros2 launch controller_node actuation.launch.py
```
```
ros2 param set /simple_pid kp 100.0 && ros2 param set /simple_pid ki 1.0 && ros2 param set /simple_pid kd 15.0
```
- The ball motion is manually initiated by publiishing to an ignition topic with the following command
```
ign topic -t "/world/world_demo/wrench" -m ignition.msgs.EntityWrench -p "entity: {name: 'ping_pong_ball::ping_pong_ball_link', type: LINK}, wrench: {force: {x:-1, z: 8}}"
```
When running the simulation, having terminals open for each execution of the launch scripts assists for debugging. Within the Ignition Gazebo gui, the otput images of the cameras are also present to facilitate debugging and plannin gof ball motion for trials. 
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
