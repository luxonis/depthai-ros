ARG ROS_DISTRO=noetic
FROM ros:${ROS_DISTRO}-ros-base
ARG USE_RVIZ
ARG BUILD_SEQUENTIAL=0
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh python3-catkin-tools

ENV DEBIAN_FRONTEND=dialog
RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"

ENV WS=/ws
RUN mkdir -p $WS/src
COPY ./ .$WS/src/depthai-ros
RUN cd .$WS/ && rosdep install --from-paths src --ignore-src -y
RUN if [ "$BUILD_SEQUENTIAL" = "1" ] ; then cd .$WS/ && . /opt/ros/noetic/setup.sh && catkin build -j1 -l1; else cd .$WS/ && . /opt/ros/noetic/setup.sh && catkin build; fi 
RUN if [ "$USE_RVIZ" = "1" ] ; then echo "RVIZ ENABLED" && sudo apt install -y ros-noetic-rviz ros-noetic-rviz-imu-plugin ; else echo "RVIZ NOT ENABLED"; fi
RUN echo "if [ -f ${WS}/devel/setup.zsh ]; then source ${WS}/devel/setup.zsh; fi" >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/devel/setup.bash ]; then source ${WS}/devel/setup.bash; fi" >> $HOME/.bashrc
ENTRYPOINT [ "/ws/src/depthai-ros/entrypoint.sh" ]
CMD ["zsh"]