FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    libeigen3-dev \
    ros-humble-rviz2 \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-visualization-msgs \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init \
    && rosdep update

WORKDIR /RRT_planner

COPY . .

RUN rosdep install --from-paths src --ignore-src -r -y

RUN source /opt/ros/humble/setup.bash && \
    colcon build

# Set up ros2 network isolation and CycloneDDS
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /RRT_planner/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]
