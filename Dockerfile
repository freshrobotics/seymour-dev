FROM docker.io/library/ubuntu:jammy

# label with source repo
LABEL org.opencontainers.image.source=https://github.com/freshrobotics/seymour-dev

ARG USERNAME="seymour"
ARG HOME_DIR="/home/${USERNAME}"
ARG WORKSPACE="${HOME_DIR}/workspace"
ARG DDS_CONFIG_DIR="/opt/dds/config"
ARG DEBIAN_FRONTEND="noninteractive"
ARG RUN_AS_UID=1000
ARG RUN_AS_GID=1000

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO="humble"
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
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" \
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

# create non-root user with given username
# and allow sudo without password
# and setup default users .bashrc
RUN groupadd --gid $RUN_AS_GID ${USERNAME} \
  && useradd -rm \
    -d ${HOME_DIR} \
    -s /bin/bash \
    --gid ${RUN_AS_GID} \
    --uid ${RUN_AS_UID} \
    -m ${USERNAME} \
  && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
  && chmod 0440 /etc/sudoers.d/${USERNAME} \
  && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME_DIR}/.bashrc \
  && echo "source install/setup.bash" >> ${HOME_DIR}/.bashrc \
  && echo "source /etc/profile.d/bash_completion.sh" >> ${HOME_DIR}/.bashrc \
  && chown -R ${USERNAME}: ${HOME_DIR}

# create workspace and source dir
RUN mkdir -p ${WORKSPACE} \
  && chown -R ${USERNAME}: ${HOME_DIR}
WORKDIR ${WORKSPACE}

# setup dds config
ADD ./dds_config ${DDS_CONFIG_DIR}

# enable either cyclone dds or fast rtps

# copy code into workspace and set ownership to user
ADD --chown=${USERNAME}:${USERNAME} ./src ${WORKSPACE}/src

# install deps as non-root user
USER ${USERNAME}
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
  && sudo apt-get update \
  && sudo rosdep init \
  && rosdep update --rosdistro ${ROS_DISTRO} \
  && rosdep install -y -r -i --from-paths ${WORKSPACE}/src \
  && sudo rm -rf /var/lib/apt/lists/*"

# RUN unset FASTRTPS_DEFAULT_PROFILES_FILE

# by default hold container open in background
CMD ["tail", "-f", "/dev/null"]
