"""
Map the warehouse for real, with SLAM, instead of trusting the generated prior.

    ros2 launch warehouse_picker slam_mapping.launch.py
    # drive amr_1 around, then:
    ros2 run nav2_map_server map_saver_cli -f ~/warehouse_slam

The map in `maps/` is derived analytically from the world's collision geometry,
which makes it exact but also makes it a description of the simulator rather
than of a warehouse. Running slam_toolbox over the Tugbot's scanner produces
the map a real robot would build -- with the occlusions, the mixed-pixel edges
at rack legs and the drift that a real deployment has to plan on top of. Having
both is the point: the generated map is ground truth to score the SLAM map
against.

Requires `ros-<distro>-slam-toolbox`. The launch degrades to just the bridge
and a teleop hint if it is not installed, rather than failing outright.
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = "world_demo"


def generate_launch_description():
    pkg = get_package_share_directory("warehouse_picker")
    world = os.path.join(pkg, "worlds", "fleet_warehouse.sdf")
    robot = "amr_1"
    scan_topic = (f"/world/{WORLD_NAME}/model/{robot}/link/scan_front"
                  f"/sensor/scan_front/scan")

    gz = ExecuteProcess(cmd=["gz", "sim", "-r", world], output="screen")

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"/model/{robot}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            f"/model/{robot}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/model/{robot}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            f"{scan_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            (f"/model/{robot}/cmd_vel", "/cmd_vel"),
            (f"/model/{robot}/odometry", "/odom"),
            (f"/model/{robot}/tf", "/tf"),
            (scan_topic, "/scan"),
        ],
        output="screen", parameters=[{"use_sim_time": True}])

    # slam_toolbox needs base_link -> laser; the Tugbot's scanner sits 0.221 m
    # forward and 0.140 m up, taken from the model's own <link> pose.
    static_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["0.221", "0", "0.1404", "0", "0", "0",
                   "base_link", f"{robot}/base_link/scan_front"],
        parameters=[{"use_sim_time": True}], output="screen")

    actions = [gz, TimerAction(period=8.0, actions=[bridge, static_tf])]

    if shutil.which("ros2") and _has_slam_toolbox():
        actions.append(TimerAction(period=10.0, actions=[Node(
            package="slam_toolbox", executable="async_slam_toolbox_node",
            name="slam_toolbox", output="screen",
            parameters=[{"use_sim_time": True, "odom_frame": "odom",
                         "base_frame": "base_link", "map_frame": "map",
                         "scan_topic": "/scan", "mode": "mapping",
                         "resolution": 0.05, "max_laser_range": 5.0,
                         "minimum_travel_distance": 0.3,
                         "minimum_travel_heading": 0.3}])]))
    else:
        actions.append(TimerAction(period=10.0, actions=[ExecuteProcess(
            cmd=["echo", "slam_toolbox is not installed: "
                 "sudo apt install ros-$ROS_DISTRO-slam-toolbox"],
            output="screen")]))

    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value=robot),
        *actions,
    ])


def _has_slam_toolbox():
    try:
        from ament_index_python.packages import get_package_share_directory as g
        g("slam_toolbox")
        return True
    except Exception:
        return False
