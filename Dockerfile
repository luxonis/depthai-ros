ARG ROS_DISTRO=kilted
FROM ros:${ROS_DISTRO}-ros-base
ARG USE_RVIZ
ARG BUILD_SEQUENTIAL=0
ARG BUILD_TESTS=0
ARG USE_DEPTHAI_CORE=1
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends software-properties-common git libusb-1.0-0-dev wget python3-colcon-common-extensions zip unzip tar

ENV DEBIAN_FRONTEND=dialog

ENV WS=/ws
RUN mkdir -p $WS/src
COPY ./ $WS/src/
RUN cd $WS/ && apt-get update \
   && case "$USE_DEPTHAI_CORE" in \
      1) test -f src/external/depthai-core/package.xml \
         || { echo "Initialize depthai-core with git submodule update --init --recursive, or set USE_DEPTHAI_CORE=0" >&2; exit 1; }; \
         mv src/external/depthai-core src/depthai-core ;; \
      0) rm -rf src/external/depthai-core ;; \
      *) echo "USE_DEPTHAI_CORE must be 0 or 1" >&2; exit 1 ;; \
   esac \
   && rosdep install --from-paths src --ignore-src -y

RUN cd $WS/ && . /opt/ros/${ROS_DISTRO}/setup.sh && ./src/build.sh -s $BUILD_SEQUENTIAL -r 1 -m 1 -t $BUILD_TESTS
RUN if [ "$USE_RVIZ" = "1" ] ; then echo "RVIZ ENABLED" && sudo apt install -y ros-${ROS_DISTRO}-rviz2 ros-${ROS_DISTRO}-rviz-imu-plugin ; else echo "RVIZ NOT ENABLED"; fi
RUN echo "if [ -f ${WS}/install/setup.bash ]; then source ${WS}/install/setup.bash; fi" >> $HOME/.bashrc
ENTRYPOINT [ "/ws/src/entrypoint.sh" ]
CMD ["bash"]
