ARG ROS_DISTRO=humble
FROM gezp/ubuntu-desktop-ros:$ROS_DISTRO

ARG ROS_DISTRO=humble
ARG GZ_VERSION=fortress


# Example of installing programs
RUN apt-get update \
  && apt-get install -y \
  nano \
  vim \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=Catcher
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# Install sdk for dynamixel
RUN apt-get update  && \
  sudo apt-get install ros-$ROS_DISTRO-dynamixel-sdk -y && \
  rm -rf /var/lib/apt/lists/*


## Install Gazebo
RUN apt-get update  && \
  wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
  apt-get update && \
  apt-get install -y gz-$GZ_VERSION ros-$ROS_DISTRO-ros-gz && \
  rm -rf /var/lib/apt/lists/* 


## Install files for the creation of URDF files within the workspace from Onshape
RUN apt-get update \
  && pip install onshape-to-robot \
  && sudo add-apt-repository ppa:openscad/releases \
  && sudo apt-get install openscad -y \
  && sudo apt-get install meshlab -y \
  && sudo apt-get install ros-humble-joint-state-publisher -y \
  && sudo apt-get install ros-humble-joint-state-publisher-gui -y \
  && rm -rf /var/lib/apt/lists/* 


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc


# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]