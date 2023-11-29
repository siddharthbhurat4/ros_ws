FROM althack/ros2:humble-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
ENV DEBIAN_FRONTEND=noninteractive
RUN apt -y install ~nros-humble-rqt* \
   && apt-get install wget \
   && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
   && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
   && apt-get update && sudo apt-get -y install ignition-fortress \
   && apt -y install ros-humble-joint-state-publisher-gui \
   && apt -y install ros-humble-xacro \
   && apt -y install ros-humble-gazebo-ros-pkgs \
   && pip3 install setuptools==58.2.0
   # && apt install ros-humble-navigation2 \
   # && apt install ros-humble-tf2-tools \
   # && apt-get install ros-humble-hector-gazebo-plugins \
   # && pip install geographiclib \
   # && apt-get install ros-humble-tf2-py \
   # && apt-get install ros-humble-tf-transformations \
   # && apt install ros-humble-robot-localization
ENV DEBIAN_FRONTEND=dialog
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
