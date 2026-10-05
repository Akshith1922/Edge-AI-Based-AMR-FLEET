#!/usr/bin/env bash
# Check this machine can run the fleet, and say exactly what to install if not.
#
#   bash tools/doctor.sh
#
# Exits 0 if everything needed is present.

ok=0
say()  { printf '  %-34s %s\n' "$1" "$2"; }
good() { say "$1" "OK  $2"; }
bad()  { say "$1" "--  $2"; ok=1; }

echo
echo "Ubuntu / OS"
if [ -r /etc/os-release ]; then
  . /etc/os-release
  good "$PRETTY_NAME" "codename ${VERSION_CODENAME:-unknown}"
else
  bad "os-release" "cannot identify this system"
fi

echo
echo "Gazebo"
GZBIN=""
if command -v gz >/dev/null 2>&1 && gz sim --versions >/dev/null 2>&1; then
  GZBIN=gz
  good "gz sim" "$(gz sim --versions 2>/dev/null | head -1)"
elif command -v ign >/dev/null 2>&1 && ign gazebo --versions >/dev/null 2>&1; then
  GZBIN=ign
  bad "ign gazebo" "$(ign gazebo --versions 2>/dev/null | head -1) -- this is Fortress"
  echo "     This project targets Garden or newer (the 'gz sim' command)."
  echo "     Either install Gazebo Garden/Harmonic, or see the Fortress note"
  echo "     in QUICKSTART.md for the two lines to change."
else
  bad "gazebo" "not installed"
  echo "     sudo apt install gz-harmonic       # Ubuntu 24.04"
  echo "     sudo apt install gz-garden         # Ubuntu 22.04"
fi

echo
echo "ROS 2"
DISTRO="${ROS_DISTRO:-}"
if [ -z "$DISTRO" ]; then
  DISTRO=$(ls /opt/ros 2>/dev/null | head -1)
fi
if [ -n "$DISTRO" ] && [ -d "/opt/ros/$DISTRO" ]; then
  good "ROS 2 $DISTRO" "/opt/ros/$DISTRO"
  [ -z "${ROS_DISTRO:-}" ] && echo "     (not sourced yet: source /opt/ros/$DISTRO/setup.bash)"
else
  bad "ROS 2" "not installed"
  echo "     https://docs.ros.org/en/humble/Installation.html"
fi

if [ -n "$DISTRO" ]; then
  for pkg in ros_gz_bridge ros_gz_sim; do
    if [ -d "/opt/ros/$DISTRO/share/$pkg" ]; then
      good "$pkg" "present"
    else
      bad "$pkg" "missing"
      echo "     sudo apt install ros-$DISTRO-${pkg//_/-}"
    fi
  done
  if [ -d "/opt/ros/$DISTRO/share/slam_toolbox" ]; then
    good "slam_toolbox" "present (optional, for the mapping run)"
  else
    say "slam_toolbox" "..  optional: sudo apt install ros-$DISTRO-slam-toolbox"
  fi
fi

echo
echo "Build tools"
if command -v colcon >/dev/null 2>&1; then
  good "colcon" "$(colcon version-check 2>/dev/null | head -1 || echo present)"
else
  bad "colcon" "missing"
  echo "     sudo apt install python3-colcon-common-extensions"
fi

echo
echo "This package"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PKG="$HERE/ros2_ws/src/warehouse_picker"
for f in maps/warehouse_plan.json maps/warehouse.pgm config/warehouse_layout.json \
         worlds/fleet_warehouse.sdf; do
  if [ -f "$PKG/$f" ]; then
    good "$f" "$(du -h "$PKG/$f" | cut -f1)"
  else
    bad "$f" "missing -- run: python3 tools/build_map.py"
  fi
done

if command -v python3 >/dev/null 2>&1; then
  good "python3" "$(python3 -V 2>&1)"
  if [ -n "$DISTRO" ]; then
    ROSPY=$(ls -d /opt/ros/$DISTRO/lib/python3.* 2>/dev/null | head -1)
    ROSPY=$(basename "${ROSPY:-python3.x}")
    HAVE="python$(python3 -c 'import sys;print(f"{sys.version_info.major}.{sys.version_info.minor}")' 2>/dev/null)"
    if [ "$ROSPY" != "python3.x" ] && [ "$ROSPY" != "$HAVE" ]; then
      bad "python version" "ROS 2 $DISTRO wants $ROSPY, 'python3' is $HAVE"
      echo "     rclpy will not import. Make python3 resolve to /usr/bin/${ROSPY}."
    fi
  fi
fi

echo
if [ $ok -eq 0 ]; then
  echo "Everything needed is here. Next:"
  echo "    cd ros2_ws && colcon build --packages-select warehouse_picker"
  echo "    source install/setup.bash"
  echo "    ros2 launch warehouse_picker fleet_warehouse.launch.py"
else
  echo "Install the items marked -- above, then run this again."
fi
echo
exit $ok
