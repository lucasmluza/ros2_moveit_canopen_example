FROM ubuntu:22.04

LABEL title="ROS2 Humble Docker Image"
LABEL authors="Lucas Matana Luza <lucas.luza@sc.senai.br>"
LABEL description="A Docker Image for developing applications based on the ROS2"
LABEL version="v0.1"

ENV DEBIAN_FRONTEND noninteractive

RUN apt update \
    && apt upgrade -y \
    && apt install -y \
    sudo \
    software-properties-common \
    locales \ 
    curl \
    python3 \
    python3-pip \
    bash-completion

# Set locale. By default, the docker has a locale set to POSIX, and it is 
# necessary to have UTF-8 support.
RUN locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8

ENV LANG en_US.UTF-8 

ENV LC_ALL en_US.UTF-8

# Add the ROS2 apt
RUN add-apt-repository universe

RUN curl \
    -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 related packages and utils packages
# Check http://packages.ros.org/ros2/ubuntu/dists/jammy/main/ for more info about the packages
RUN apt update \
    && apt install -y \
    ros-humble-ros-base \
    ros-humble-rviz2 \
    ros-humble-rqt* \
    ros-dev-tools \
    python3-rosdep \
    python3-colcon-common-extensions \
    can-utils \
    && rm -rf /var/lib/apt/lists/*

# Add non-root user
RUN addgroup --gid 1000 ros2_user \
    && adduser --uid 1000 --gid 1000 --disabled-password --gecos "" ros2_user \
    && echo 'ros2_user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers \
    && chmod -R 775 /home/ros2_user \
    && chown -R ros2_user:ros2_user /home/ros2_user

USER ros2_user

# Export ros2 setup.bash to .bashrc to ease the tools access
RUN echo "source /opt/ros/humble/setup.bash\n" >> /home/ros2_user/.bashrc \
    && echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash\n" >> /home/ros2_user/.bashrc

# Avoid "QStandardPaths: XDG_RUNTIME_DIR not set" warning when runing, e.g., rviz2
ENV XDG_RUNTIME_DIR=/tmp/runtime-ros2_user

RUN mkdir -p /home/ros2_user/ws_ros/src

WORKDIR /home/ros2_user/ws_ros
