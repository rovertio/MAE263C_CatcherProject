# Reference for running the system
## Requirements:
- Install Docker Desktop
- Install WSL2
- Install Ubuntu for WSL2 through microsoft store or command line
- Install VcXsrv through the site
- Go through the prompted setups for the VcXsrv executor

## Building the container
- Build the docker image
- Compose the container (run within the dockerros directory)
```
docker compose up
```
# Reference commands
- Sourcing ROS2 distro
```
source /opt/ros/humble/setup.bash
```
- Sourcing the isntall (within the ROS2 workspace directory)
```
source install/setup.bash
```
