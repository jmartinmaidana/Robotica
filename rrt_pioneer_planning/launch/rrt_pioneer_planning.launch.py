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
            package='rrt_pioneer_planning',
            executable='rrt_pioneer_planning',
            name='rrt_pioneer_planning',
            output='screen',
            parameters=[{"use_sim_time": True},{
                'goal_bias': 0.2,
                'max_iterations': 20000,
                'linear_velocity_stepping': 0.5,
                'angular_velocity_stepping': 0.25,
            }]
        ),

        #Trajectory follower (close)
        Node(
            package="lazo_cerrado",
            executable="trajectory_follower_cl",
            name="trajectory_follower_cl",
            output="screen",
            parameters=[
                {'use_sim_time': True},
                {"goal_selection": "PURSUIT_BASED"}, #FIXED_GOAL, TIME_BASED, PURSUIT_BASED
            ],
        ),
    ])