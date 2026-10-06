#!/bin/bash

# followed this guide: https://ryan.himmelwright.net/post/scripting-tmux-workspaces/
session="ros_sim"

# Kill old session if it exists (2>/dev/null is for slicencing error message in case
# session is not real)
tmux kill-ses -t $session 2>/dev/null

# Create new session
tmux new-session -d -s $session
tmux rename-window -t 0 'Main'

tmux send-keys -t 'Main' 'colcon build --cmake-args -DBUILD_TESTING=ON' c-m
tmux send-keys -t 'Main' 'source install/setup.bash' c-m
tmux send-keys -t 'Main' 'ros2 launch ros_gz_example_bringup diff_drive.launch.py' c-m

# Create terminal window
tmux new-window -t $session:2 -n 'Terminal'

# attatch to current session to be in it's space
tmux attach -d -t $session

tmux send-keys -t 'Bridge' 'echo "to exit hit ctrl-b then type ':kill-session'\n To switch between pannels perform ctrl-b {window index}"' c-m
