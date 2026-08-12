ARG BASE_IMAGE=ghcr.io/ros-controls/ros:humble-ubuntu-testing
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND noninteractive

RUN mkdir -p /home/ros2_ws/src \
    && cd /home/ros2_ws/src \
    && if [ "${ROS_DISTRO}" = "rolling" ] ; then \
      git clone https://github.com/ros-controls/gz_ros2_control/; \
    else \
      git clone https://github.com/ros-controls/gz_ros2_control/ -b ${ROS_DISTRO}; \
    fi \
    && rosdep update \
    && apt-get update \
    && rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}

RUN cd /home/ros2_ws/ \
  && . /opt/ros/"${ROS_DISTRO}"/setup.sh \
  && colcon build --merge-install

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ros2 launch gz_ros2_control_demos cart_example_position.launch.py
