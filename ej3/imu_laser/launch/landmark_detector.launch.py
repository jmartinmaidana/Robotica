from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Pioneer odometry node
        Node(
            package="modelo_diferencial",
            executable="pioneer_odometry_node",  # check executable name in ROS 2
            name="pioneer_odometry",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),

        #Landmark detector
        Node(
            package="imu_laser",
            executable="landmark_detector",
            name="landmark_detector",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),
    ])