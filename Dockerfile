# Simulator image: Gazebo (Ignition Fortress) + ROS 2 Humble + the Overhaul packages,
# built and ready to launch, with a VNC desktop for the GUI (Gazebo, RViz).
# Based on Overhaul's Dockerfile (build/entrypoint) and .devcontainer/Dockerfile (VNC desktop).
FROM ros:humble

ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Non-root user with passwordless sudo (replaces any existing user with the same UID)
RUN if getent passwd "$USER_UID"; then \
        userdel -r "$(getent passwd "$USER_UID" | cut -d: -f1)"; \
    fi \
    && groupadd --gid "$USER_GID" "$USERNAME" \
    && useradd --uid "$USER_UID" --gid "$USER_GID" -m -s /bin/bash "$USERNAME" \
    && apt-get update \
    && apt-get install -y --no-install-recommends sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/"$USERNAME" \
    && chmod 0440 /etc/sudoers.d/"$USERNAME" \
    && rm -rf /var/lib/apt/lists/*

# Desktop + VNC server, and noVNC so the desktop is also reachable from a browser
RUN apt-get update && apt-get install -y --no-install-recommends \
    xfce4 \
    xfce4-terminal \
    tigervnc-standalone-server \
    tigervnc-common \
    tigervnc-tools \
    novnc \
    python3-websockify \
    dbus-x11 \
    fonts-liberation \
    xfonts-base \
    mesa-utils \
    tmux \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Simulator runtime: ros_gz pulls in Ignition Fortress; RViz and robot_state_publisher are used by the launch files
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-gz \
    ros-humble-rviz2 \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-sdformat-urdf \
    && rm -rf /var/lib/apt/lists/*

# Rerun C++ SDK for streaming the robot to a Rerun viewer on the host (urdf_to_rerun).
# Built once here (it compiles a minimal Apache Arrow) and installed to /opt/rerun_cpp_sdk,
# where ros_gz_example_bringup finds it. Keep RERUN_VERSION equal to the viewer's.
ARG RERUN_VERSION=0.28.1
RUN apt-get update && apt-get install -y --no-install-recommends curl unzip \
    && rm -rf /var/lib/apt/lists/* \
    && cd /tmp \
    && curl -fsSL -o rerun_cpp_sdk.zip \
        "https://github.com/rerun-io/rerun/releases/download/${RERUN_VERSION}/rerun_cpp_sdk.zip" \
    && unzip -q rerun_cpp_sdk.zip \
    && cmake -S rerun_cpp_sdk -B rerun_build -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/opt/rerun_cpp_sdk -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    && cmake --build rerun_build -j"$(nproc)" \
    && cmake --install rerun_build \
    && rm -rf /tmp/rerun_cpp_sdk.zip /tmp/rerun_cpp_sdk /tmp/rerun_build

# No GPU in the container: render Gazebo/RViz with Mesa's software rasterizer
ENV LIBGL_ALWAYS_SOFTWARE=1 \
    DISPLAY=:1 \
    SHELL=/bin/bash

WORKDIR /ros2_ws
RUN chown "$USERNAME":"$USERNAME" /ros2_ws
USER $USERNAME

# Install the remaining package dependencies, then build the workspace (simulator + Overhaul)
COPY --chown=$USERNAME:$USERNAME src/ src/
RUN sudo apt-get update -q \
    && rosdep update --rosdistro "$ROS_DISTRO" \
    && rosdep install --from-paths src --ignore-src -y \
    && sudo rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/"$ROS_DISTRO"/setup.bash \
    && colcon build

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc \
    && echo '[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash' >> ~/.bashrc

COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh
COPY --chmod=755 docker/start-desktop.sh /usr/local/bin/start-desktop

# 5901: VNC, 6080: noVNC (browser)
EXPOSE 5901 6080

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ros_gz_example_bringup", "diff_drive.launch.py"]
