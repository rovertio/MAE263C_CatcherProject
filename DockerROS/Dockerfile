ARG ROS_DISTRO=humble
FROM gezp/ubuntu-desktop-ros:$ROS_DISTRO

# Remove old ROS sources and keys, then add the new key and source
RUN rm -f /etc/apt/sources.list.d/ros2.list && \
    rm -f /etc/apt/trusted.gpg.d/ros* && \
    apt-get update && apt-get install -y curl gnupg2 && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
ARG ROS_DISTRO=humble
ARG GZ_VERSION=fortress

# Remove or comment out this block:
# # Create a non-root user
# ARG USERNAME=Catcher
# ARG USER_UID=1000
# ARG USER_GID=$USER_UID
# 
# RUN groupadd --gid $USER_GID $USERNAME \
#   && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
#   && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo (already installed in most base images, so this can be omitted)
# RUN apt-get update \
#   && apt-get install -y sudo \
#   && rm -rf /var/lib/apt/lists/*


# Installing utilities
RUN apt-get update && \
  # Installing text editors
  apt-get install nano -y && \
  apt-get install vim -y && \
  # Installing OpenCV
  apt-get install libopencv-dev -y && \
  # Installing python dependencies
  apt-get install python3-numpy -y && \
  apt-get install libboost-python-dev -y && \
  # Installing cpp compiler (if want to use instead of deault g++)
  apt-get install clang -y && \
  rm -rf /var/lib/apt/lists/*


## Install ROS2 related items
RUN apt-get update  && \
  # Install Gazebo
  wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
  apt-get update && \
  apt-get install -y gz-$GZ_VERSION ros-$ROS_DISTRO-ros-gz && \
  apt-get install -y ros-${ROS_DISTRO}-ros-ign-bridge && \
  # Install package for joint control in rviz
  apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher && \
  apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher-gui && \
  # Install cv_bridge
  apt-get install -y ros-${ROS_DISTRO}-cv-bridge && \
  apt-get install -y ros-${ROS_DISTRO}-vision-opencv && \
  # Install package for dynamixel
  apt-get install -y ros-$ROS_DISTRO-dynamixel-sdk && \
  # Install ament_cmake for building CMake-based ROS2 packages
  apt-get install -y ros-${ROS_DISTRO}-ament-cmake && \
  rm -rf /var/lib/apt/lists/* 


## Install files for the creation of URDF files within the workspace from Onshape
RUN apt-get update && \
  pip install onshape-to-robot && \
  add-apt-repository ppa:openscad/releases && \
  apt-get install -y openscad && \
  apt-get install -y meshlab && \
  rm -rf /var/lib/apt/lists/* 


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc