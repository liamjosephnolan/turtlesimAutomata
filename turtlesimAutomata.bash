#!/bin/bash

# Define the source and destination paths
SOURCE=$(pwd)
DESTINATION=~/ros2_ws/src

# Ensure the turtlesimAutomata directory exists in the current directory
if [ ! -d "./turtlesimAutomata" ]; then
    echo "Error: Source directory './turtlesimAutomata' does not exist."
fi

# Make the destination directory if it does not exist
mkdir -p "$DESTINATION"

# Copy the directory to the destination directory
cp -rf "$SOURCE/turtlesimAutomata" "$DESTINATION"

# Verify the copy was successful
if [ ! -d "$DESTINATION/turtlesimAutomata" ]; then
    echo "Error: Copy failed. Directory '$DESTINATION/turtlesimAutomata' does not exist."
fi

# Change to the ros2_ws directory
cd ~/ros2_ws

# Remove the source directory
rm -rf "$SOURCE"

# Run turtle sim node
gnome-terminal -- bash -c "source ~/.bashrc; cd ~/ros2_ws; ros2 run turtlesim turtlesim_node"

# Run Edge detection node
gnome-terminal -- bash -c "cd ~/ros2_ws; sleep .5; colcon build --packages-select turtlesimAutomata; source install/setup.bash; ros2 run turtlesimAutomata edge_detect"

