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
tmux send-keys -t ros_sessions:1 "roslaunch cse478 teleop.launch map:='\$(find mushr_sim)/maps/sandbox.yaml'" C-m

# Create the second window and wait for prompt
tmux new-window -t ros_sessions:2 -n "path_publisher"
wait_for_prompt "ros_sessions:2"
tmux send-keys -t ros_sessions:2 "roslaunch introduction path_publisher.launch plan_file:='\$(find introduction)/plans/figure_8.txt'" C-m

# Create the third window and wait for prompt
tmux new-window -t ros_sessions:3 -n "rviz"
wait_for_prompt "ros_sessions:3"
tmux send-keys -t ros_sessions:3 "rosrun rviz rviz -d ~/mushr_ws/src/mushr478/cse478/config/default.rviz" C-m

# Attach to the tmux session
tmux attach-session -t ros_sessions
