FROM althack/ros2:humble-dev 

ENV DEBIAN_FRONTEND=noninteractive
RUN apt -y install ~nros-humble-rqt* \
   && apt-get install wget \
   && sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
   && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
   && apt-get update && sudo apt-get -y install ignition-fortress \
   && apt -y install ros-humble-joint-state-publisher-gui \
   && apt -y install ros-humble-xacro \
   && apt -y install ros-humble-gazebo-ros-pkgs \
   && pip3 install setuptools==58.2.0 \
   && apt -y install ros-humble-navigation2 \
   && apt -y install ros-humble-tf2-tools \
   && apt-get -y install ros-humble-tf2-py \
   && apt-get -y install ros-humble-tf-transformations \
   && pip3 install transforms3d \
   && pip3 install matplotlib 
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
