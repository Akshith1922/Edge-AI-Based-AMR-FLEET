"""
Bring up the warehouse, the fleet and the dashboard.

    ros2 launch warehouse_picker fleet_warehouse.launch.py
    ros2 launch warehouse_picker fleet_warehouse.launch.py robots:=3 gui:=false

What changed from the version that would not drive autonomously:

* The robots are spawned by the world file, not by `ros_gz_sim create` here as
  well. Spawning them twice produced a second entity that Gazebo renamed, and a
  renamed entity publishes on `/model/amr_1_0/...`, so every bridge below
  silently pointed at topics nothing was writing to.
* The Tugbot's `scan_front` lidar is bridged. Without it the agent had no
  perception at all and could only avoid robots that told it where they were --
  it drove through racks because as far as it knew there were none.
* Bridge names are derived from the world name, because a gz sensor topic
  embeds it: `/world/<world>/model/<robot>/link/...`. Hard-coding the world
  name is the usual reason a scan bridge comes up subscribed to nothing.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            OpaqueFunction, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WORLD_NAME = "world_demo"       # must match <world name=...> in the SDF


def launch_setup(context, *_args, **_kwargs):
    pkg = get_package_share_directory("warehouse_picker")
    world = os.path.join(pkg, "worlds", "fleet_warehouse.sdf")
    plan_map = os.path.join(pkg, "maps", "warehouse_plan.json")
    layout = os.path.join(pkg, "config", "warehouse_layout.json")

    count = int(LaunchConfiguration("robots").perform(context))
    policy = LaunchConfiguration("policy").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")
    speed = float(LaunchConfiguration("max_speed").perform(context))
    tasks = int(LaunchConfiguration("tasks").perform(context))
    names = [f"amr_{i + 1}" for i in range(count)]

    gz = ExecuteProcess(
        cmd=["gz", "sim", "-r"] + ([] if gui else ["-s"]) + [world],
        output="screen")

    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
    remaps = []
    for name in names:
        scan_topic = (f"/world/{WORLD_NAME}/model/{name}/link/scan_front"
                      f"/sensor/scan_front/scan")
        bridge_args += [
            f"/model/{name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            f"/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"{scan_topic}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ]
        remaps += [
            (f"/model/{name}/cmd_vel", f"/{name}/cmd_vel"),
            (f"/model/{name}/odometry", f"/{name}/odom"),
            (scan_topic, f"/{name}/scan"),
        ]

    bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                  arguments=bridge_args, remappings=remaps, output="screen",
                  parameters=[{"use_sim_time": True}])

    agents = [
        Node(package="warehouse_picker", executable="edge_agent",
             name=f"{name}_edge_agent", output="screen",
             parameters=[{"robot_id": name, "map_path": plan_map,
                          "policy": policy, "max_speed": speed,
                          "use_sim_time": True}])
        for name in names
    ]

    dispatcher = Node(
        package="warehouse_picker", executable="dispatcher", output="screen",
        parameters=[{"tasks": tasks, "map_path": plan_map,
                     "layout_path": layout, "use_sim_time": True}])

    dashboard = Node(
        package="warehouse_picker", executable="dashboard", output="screen",
        parameters=[{"port": 8080, "assets": pkg, "use_sim_time": True}])

    return [
        gz,
        # Gazebo has to have finished loading the world -- and pulling the Fuel
        # models on a cold cache -- before anything subscribes to its topics.
        TimerAction(period=8.0, actions=[bridge]),
        TimerAction(period=12.0, actions=agents + [dispatcher, dashboard]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robots", default_value="3",
                              description="how many AMRs to drive (they must "
                                          "exist in the world SDF)"),
        DeclareLaunchArgument("policy", default_value="cooperative",
                              description="cooperative | stop_and_wait"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("max_speed", default_value="0.8"),
        DeclareLaunchArgument("tasks", default_value="12"),
        OpaqueFunction(function=launch_setup),
    ])
