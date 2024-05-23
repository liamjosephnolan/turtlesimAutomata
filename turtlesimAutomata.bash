#!/bin/sh

# Make directory
sudo mkdir ~/ros2_ws/src

# Copy Content
sudo cp -r ~/Downloads/turtlesimAutomata ~/ros2_ws/src

# Remove folder and files
sudo rm -r ~/Downloads/turtlesimAutomata

# Run turtle sim node
gnome-terminal -- bash -c "source ~/.bashrc;cd ~/ros2_ws;ros2 run turtlesim turtlesim_node"

# Run Edge detection node
gnome-terminal -- bash -c "cd ~/ros2_ws; sleep .5; colcon build --packages-select turtlesimAutomata; source install/setup.bash;ros2 run turtlesimAutomata edge_detect
"

