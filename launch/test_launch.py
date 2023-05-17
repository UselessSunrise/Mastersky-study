from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(cmd=[["ign gazebo --render-engine ogre ~/ros2_ws/src/py_pubsub/worlds/diff_drive.sdf"]], shell=True),
            # ros2 run ros_ign_bridge parameter_bridge /model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist /lidar_blue@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan
            Node(
                package="ros_ign_bridge",
                executable="parameter_bridge",
                arguments=[
                    "/model/vehicle_blue/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist "
                    + "/vehicle_blue_lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan "
                ],
            ),
            # Node(
            #     package='ros_ign_bridge',
            #     executable='parameter_bridge',
            #     arguments=['/model/vehicle_green/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
            # ),
            Node(
                package="py_pubsub",
                executable="talker",
            ),
        ]
    )
