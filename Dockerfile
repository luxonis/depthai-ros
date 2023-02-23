ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base
ARG USE_RVIZ
ARG BUILD_SEQUENTIAL=1
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget zsh python3-colcon-common-extensions
# using cyclone since there are some problems with discovery using fastrps & docker
RUN apt install -y ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
ENV DEBIAN_FRONTEND=dialog
RUN sh -c "$(wget https://raw.github.com/ohmyzsh/ohmyzsh/master/tools/install.sh -O -)"
RUN apt install libopencv-dev -y
RUN git clone --recursive https://github.com/luxonis/depthai-core.git --branch rvc3_support
RUN cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
RUN cmake --build depthai-core/build --target install
RUN rm -r depthai-core
ENV WS=/ws
RUN mkdir -p $WS/src
COPY ./ .$WS/src/depthai-ros
RUN cd .$WS/ && rosdep install --from-paths src --ignore-src  -y --skip-keys depthai
RUN cd .$WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && ./src/depthai-ros/build.sh -s $BUILD_SEQUENTIAL -r 1 -m 1 
RUN if [ "$USE_RVIZ" = "1" ] ; then echo "RVIZ ENABLED" && sudo apt install -y ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-rviz-imu-plugin ; else echo "RVIZ NOT ENABLED"; fi
RUN echo "if [ -f ${WS}/install/setup.zsh ]; then source ${WS}/install/setup.zsh; fi" >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 ros2)"' >> $HOME/.zshrc
RUN echo 'eval "$(register-python-argcomplete3 colcon)"' >> $HOME/.zshrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $HOME/.zshrc
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> $HOME/.bashrc
ENTRYPOINT [ "/ws/src/depthai-ros/entrypoint.sh" ]
CMD ["zsh"]
