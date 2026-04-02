# !/bin/bash

# Function to wait for the prompt cursor
wait_for_prompt() {
  # Wait until we detect the prompt (usually the default bash prompt)
  tmux send-keys -t "$1" C-m  # Ensure the prompt is fully loaded
  while true; do
    # Check if the prompt is available by looking for a prompt (e.g., $ or #)
    if tmux capture-pane -t "$1" -p | grep -q -E '[$#]'; then
      break
    fi
    sleep 0.5  # Wait for half a second before checking again
  done
}

# Create a new tmux session with the name "ros_sessions" in detached mode
tmux new-session -d -s ros_sessions

# Create the first window and wait for prompt
tmux new-window -t ros_sessions:1 -n "teleop"
wait_for_prompt "ros_sessions:1"
tmux send-keys -t ros_sessions:1 "roslaunch mushr_base teleop.launch" C-m

# Create the second window and wait for prompt
tmux new-window -t ros_sessions:2 -n "map"
wait_for_prompt "ros_sessions:2"
tmux send-keys -t ros_sessions:2 "rosrun map_server map_server \$(rospack find cse478)/maps/002.yaml" C-m

# Create the third window and wait for prompt
tmux new-window -t ros_sessions:3 -n "pf"
wait_for_prompt "ros_sessions:3"
tmux send-keys -t ros_sessions:3 "roslaunch localization particle_filter_sim.launch use_namespace:=true publish_tf:=true tf_prefix:="car/"" C-m

# Create the fourth window and wait for prompt
tmux new-window -t ros_sessions:4 -n "controller"
wait_for_prompt "ros_sessions:4"
tmux send-keys -t ros_sessions:4 "roslaunch control controller.launch type:=pid tf_prefix:="car/"" C-m

# Create the fifth window and wait for prompt
tmux new-window -t ros_sessions:5 -n "planner"
wait_for_prompt "ros_sessions:5"
tmux send-keys -t ros_sessions:5 "roslaunch planning planner_car.launch num_vertices:=1000 connection_radius:=20 curvature:=1" C-m

# Attach to the tmux session
tmux attach-session -t ros_sessions
