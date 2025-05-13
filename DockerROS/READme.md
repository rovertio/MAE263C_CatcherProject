vcxsrv# Reference for running the system
## Requirements:
- Install Docker Desktop
- Install WSL2
- Install Ubuntu for WSL2 through microsoft store or command line
- Install VcXsrv through the site [Download link](https://vcxsrv.com/)
- Go through the prompted setups for the VcXsrv executor

## Building the container:
- Build the docker image
- Compose the container (run within the dockerros directory)
```
docker compose up
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
# Refrence media:
