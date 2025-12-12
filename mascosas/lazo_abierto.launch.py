from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Use simulation time for all nodes
        Node(
            package='modelo_diferencial',
            executable='pioneer_odometry_node',
            name='pioneer_odometry',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='lazo_abierto',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'stepping': 0.1},
                {'trajectory_type': 'spline'},
                {'total_time': 20.0},
                {'amplitude': 1.0},
                {'cycles': 1.0},
                {'spline_waypoints': [
                    0., 0., 0., 0.,     # Inicio en el origen
                    100., 5., 0., 1.57,  # Primer vértice hacia la diagonal
                    200., 0., 0., 3.14,    # Segundo vértice hacia el eje X
                    300., -5., 0., 4.71,  # Tercer vértice hacia abajo
                    400., 0., 0., 6.28     # Vuelta al eje X
                ]}

                # {'spline_waypoints': [
                #     0., 0., 0., 0.,
                #     100., 5., 0., 1.57,
                #     200., 5., 5., 3.14,
                #     300., 0., 5., 4.71,
                #     400., 0., 0., 0.
                # ]}
            ]
        )
    ])
# Note: each waypoint must have 4 values: time(sec), position_x(m), position_y(m), orientation(rad)