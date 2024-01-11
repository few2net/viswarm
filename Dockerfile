# Use the MoveIt2 image as the base
FROM ros:humble-ros-core-jammy as stage1
WORKDIR /root
# Copy each ROS2 workspace into the container
COPY ./ws_moveit2 /root/ws_moveit2/
COPY ./turtlebot3_ws /root/turtlebot3_ws/
COPY ./tb_custom_ws /root/tb_custom_ws/
COPY ./test_ws /root/test_ws/
# Install additional dependencies if needed
RUN . /opt/ros/humble/setup.sh &&\
    apt update && apt upgrade -y && apt install -y \
    tmux git nano\
    python3-argcomplete python3-colcon-common-extensions libboost-system-dev build-essential \
    python3-rosdep python3-vcstool python3-colcon-mixin\
    ros-humble-hls-lfcd-lds-driver ros-humble-turtlebot3-msgs \
    ros-humble-dynamixel-sdk libudev-dev &&\
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml &&\
    colcon mixin update default &&\
    apt update && \
    rosdep init &&\
    rosdep update &&\
    echo "source /opt/ros/humble/setup.sh" >> /root/.bashrc
CMD ["bash"]


FROM stage1 as stage2
WORKDIR /root
RUN . /opt/ros/humble/setup.sh &&\
    cd /root/ws_moveit2/src &&\
    rosdep install --from-paths ./ --ignore-src --rosdistro humble -r -y

RUN . /opt/ros/humble/setup.sh &&\
    cd /root/ws_moveit2/ &&\
    colcon build --mixin release &&\
    echo "source /root/ws_moveit2/install/setup.bash" >> /root/.bashrc
CMD ["bash"]


FROM stage2 as stage3
WORKDIR /root
RUN . /opt/ros/humble/setup.sh &&\
    . /root/ws_moveit2/install/setup.sh &&\
    cd /root/turtlebot3_ws/src &&\
    rosdep install --from-paths ./ --ignore-src --rosdistro humble -r -y

RUN . /opt/ros/humble/setup.sh &&\
    . /root/ws_moveit2/install/setup.sh &&\
    cd /root/turtlebot3_ws/ &&\
    colcon build --symlink-install &&\
    echo "source /root/turtlebot3_ws/install/setup.bash" >> /root/.bashrc
CMD ["bash"]


FROM stage3 as stage4
WORKDIR /root
RUN . /opt/ros/humble/setup.sh &&\
    . /root/ws_moveit2/install/setup.sh &&\
    cd /root/tb_custom_ws/src &&\
    rosdep install --from-paths ./ --ignore-src --rosdistro humble -r -y
RUN . /opt/ros/humble/setup.sh &&\
    . /root/ws_moveit2/install/setup.sh &&\
    cd /root/tb_custom_ws/ &&\
    colcon build --symlink-install &&\
    echo "source /root/tb_custom_ws/install/setup.bash" >> /root/.bashrc
CMD ["bash"]


FROM stage4 as stage5
WORKDIR /root
RUN . /opt/ros/humble/setup.sh &&\
    . /root/ws_moveit2/install/setup.sh &&\
    cd /root/test_ws/src &&\
    rosdep install --from-paths ./ --ignore-src --rosdistro humble -r -y
RUN . /opt/ros/humble/setup.sh &&\
    . /root/ws_moveit2/install/setup.sh &&\
    cd /root/test_ws/ &&\
    colcon build --symlink-install --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release &&\
    rm -rf /var/lib/apt/lists/* &&\
    echo "source /root/test_ws/install/setup.bash" >> /root/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle_pi" >> /root/.bashrc &&\
    echo "export ROS_DOMAIN_ID=30 #TURTLEBOT3" >> /root/.bashrc &&\
    echo "export LDS_MODEL=LDS-01" >> /root/.bashrc &&\
    echo "export OPENCR_PORT=/dev/ttyACM0" >> /root/.bashrc &&\
    echo "export OPENCR_MODEL=turtlebot3_manipulation" >> /root/.bashrc
CMD ["bash"]

