# Prereq

Sim won't be running on docker as it only needs to run on the vms thus install ROS locally following this [tutorial](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html). Select binary install version. Stop at Environment setup.

Note that if you are having issues with the vm network check if the NAT option is selected in the VM options. Also verify that the NAT process in running in 'ctrl + r' and type in 'services.msc'. Start it if it is not running to get internet connection.

Now Install: https://gazebosim.org/docs/fortress/install/ 

# Docker

The simulator can also run in a container (ROS 2 Humble + Ignition Fortress + the Overhaul packages). It's based on the Docker setup in the Overhaul repo. The GUI (Gazebo, RViz) runs in a VNC desktop with software rendering, so no GPU or X server is needed on the host.

```bash
docker compose up --build
```

Then open http://localhost:6080/vnc.html in a browser, or point a VNC client at `localhost:5901`. The password is `dockervnc`; set `VNC_PASSWORD` to change it.

- By default this launches `diff_drive.launch.py`. For the sim with the Overhaul rover/drive nodes, run `docker compose run --rm --service-ports sim ros2 launch rover rover_sim_launch.py`
- To get a shell instead: `docker compose run --rm --service-ports sim bash` (the workspace is already built and sourced)
- To open a second terminal in a running container: `docker compose exec sim bash`
- The image is built from `src/`, so rebuild it (`--build`) after code changes. For iterative work, use the VS Code devcontainer (*Reopen in Container*), which bind-mounts the repo and builds it in place.

# Build/ Using

1. `cd /Overhaul_Simulator`

2. `source /opt/ros/humble/setup.bash`\
    -   If you are in the vmware VM use the instruction `. ~/ros2_humble/install/local_setup.bash`

3. `colcon build --cmake-args -DBUILD_TESTING=ON`

4. `source install/setup.bash`

5. `ros2 launch ros_gz_example_bringup diff_drive.launch.py`

# Shortcut commands
The following bash script `./run_all.sh` runs all the 

# Testing
To send a raw command over the cli to the active node you can do `ign topic -t "/Direction" -m ignition.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"`

# Testing bridge
Run this command in another terminal (have gazebo sim running as well). This command sends the direction value over to the gazebo side from the ros communictation stack.

`ros2 run ros_gz_bridge parameter_bridge /Direction@geometry_msgs/msg/Twist@ignition.msgs.Twist`

A ros command that should be sent throuhg the bridge

`ros2 topic pub /Direction/BackLeft geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 2.0, y: 0.0, z: 0.0}}"`

