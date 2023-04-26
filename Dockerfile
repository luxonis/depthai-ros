ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
ARG USE_RVIZ
ARG BUILD_SEQUENTIAL=0
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh python3-colcon-common-extensions


ENV DEBIAN_FRONTEND=dialog
RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"

ENV WS=/ws
RUN mkdir -p $WS/src
COPY ./ .$WS/src/depthai-ros
RUN cd .$WS/ && rosdep install --from-paths src --ignore-src  -y

RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && ./src/depthai-ros/build.sh -s $BUILD_SEQUENTIAL -r 1 -m 1 
RUN if [ "$USE_RVIZ" = "1" ] ; then echo "RVIZ ENABLED" && sudo apt install -y ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-rviz-imu-plugin ; else echo "RVIZ NOT ENABLED"; fi
RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
ENTRYPOINT [ "/ws/src/depthai-ros/entrypoint.sh" ]
CMD ["zsh"]