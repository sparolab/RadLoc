#!/bin/bash
# Sets up the ROS 2 environment, the workspace overlay when one has been built,
# and a few helpers, then hands over to the command.
set -e

source /opt/ros/humble/setup.bash
if [ -f "$RADLOC_ROS_INSTALL/setup.bash" ]; then
  source "$RADLOC_ROS_INSTALL/setup.bash"
fi

# Rebuild the C++ core against the mounted sources.
radloc-build-core() {
  cmake -S /radloc/cpp/radloc -B "$RADLOC_CORE_BUILD" -DCMAKE_BUILD_TYPE=Release
  cmake --build "$RADLOC_CORE_BUILD" -j"$(nproc)"
}

# Build the ROS 2 workspace, if one is mounted.
radloc-build() {
  if [ ! -d /radloc/ros2_ws/src ]; then
    echo "no /radloc/ros2_ws/src - nothing to build" >&2
    return 0
  fi
  # colcon writes its log next to the working directory, and every path it is
  # given must be outside the read-only source mount.
  cd /opt/radloc
  colcon --log-base "$RADLOC_ROS_LOG" build --symlink-install \
    --base-paths /radloc/ros2_ws/src \
    --build-base "$RADLOC_ROS_BUILD" \
    --install-base "$RADLOC_ROS_INSTALL" \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DRADLOC_BUILD_TOOLS=OFF
  source "$RADLOC_ROS_INSTALL/setup.bash"
}

# Run every check against the mounted datasets.
radloc-validate() {
  bash /radloc/docker/validate.sh "$@"
}

export -f radloc-build-core radloc-build radloc-validate
exec "$@"
