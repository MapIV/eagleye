FROM ros:humble

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

# Create non-root user
RUN groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-vcstool \
    python3-colcon-common-extensions \
    libgeographic-dev \
    geographiclib-tools \
    geographiclib-doc \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Source ROS in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
