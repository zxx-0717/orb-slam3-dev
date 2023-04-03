FROM althack/ros2:humble-dev AS dev

ENV DEBIAN_FRONTEND=noninteractive
COPY "files/ros2.list" "/etc/apt/sources.list.d/"
COPY "files/sources.list" "/etc/apt/"
RUN apt-get update \
    && apt-get -y install --no-install-recommends \
      ros-humble-rmw-cyclonedds-cpp \
      libopencv-dev  zip bash-completion libeigen3-dev libboost-all-dev libxrandr-dev libxinerama-dev libsdl2-dev\
    && apt-get autoremove -y \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*
RUN pip install rosdepc \
    && rosdepc init
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc
RUN git config --global --add safe.directory $WORKSPACE
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp