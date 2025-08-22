#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Create resource directory if it doesn't exist
mkdir -p /ros2_ws/src/polaris_ultrasound/resource
touch /ros2_ws/src/polaris_ultrasound/resource/polaris_ultrasound

# Build the workspace if the package isn't built yet
if [ ! -d "/ros2_ws/install/polaris_ultrasound" ]; then
  echo "Building ROS2 workspace..."
  cd /ros2_ws
  colcon build --symlink-install
  echo "Workspace built successfully!"
fi

# Source the workspace
source /ros2_ws/install/setup.bash

# Execute the command passed to the entrypoint
exec "$@"