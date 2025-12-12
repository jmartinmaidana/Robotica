from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package="imu_laser",
            executable="imu_calibrator",
            name="imu_calibrator",
            output="screen",
            parameters=[{"use_sim_time": True},
                        {"calibrate": 10}]
        ),
    ])