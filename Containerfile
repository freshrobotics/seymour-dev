FROM docker.io/library/ubuntu:noble-20250714

# label with source repo
LABEL org.opencontainers.image.source=https://github.com/freshrobotics/seymour-dev-base

ARG DDS_CONFIG_DIR="/opt/dds/config"
ARG DEBIAN_FRONTEND="noninteractive"

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO="jazzy"
ENV CYCLONEDDS_URI="${DDS_CONFIG_DIR}/cyclonedds.xml"
ENV FASTRTPS_DEFAULT_PROFILES_FILE="${DDS_CONFIG_DIR}/fastrtps.xml"

# rmw implementation can be overridden at runtime
# RMW_IMPLEMENTATION -> "rmw_cyclonedds_cpp" | "rmw_fastrtps_cpp"
ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# setup utc timeszone & install base ubuntu packages
RUN echo 'Etc/UTC' > /etc/timezone  \
  && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
  && apt-get update && apt-get install -q -y --no-install-recommends \
  bash-completion \
  build-essential \
  dirmngr \
  git \
  gnupg2 \
  python-is-python3 \
  python3-pip \
  sudo \
  tzdata \
  x11-apps \
  && rm -rf /var/lib/apt/lists/*

# setup ros package overlay & install ros packages
RUN echo "deb http://packages.ros.org/ros2/ubuntu noble main" \
  > /etc/apt/sources.list.d/ros2-latest.list \
  && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 \
  --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  && apt-get update && apt-get install -q -y --no-install-recommends \
  python3-colcon-common-extensions \
  python3-colcon-mixin \
  python3-rosdep \
  python3-vcstool \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
  ros-${ROS_DISTRO}-desktop \
  && rm -rf /var/lib/apt/lists/*

# setup colcon mixin and metadata
RUN colcon mixin add default \
  https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
  colcon mixin update default && \
  colcon metadata add default \
  https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
  colcon metadata update

# remove default 'ubuntu' user account
RUN deluser --remove-home ubuntu

# by default hold container open in background
CMD ["tail", "-f", "/dev/null"]
