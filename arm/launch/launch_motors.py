from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talon',
            name='Talon1',
            executable='talon_node',
            parameters=[
                {"motor_number": 1},
                {"diagnostics_port": 56715},
                {"invert_motor": True},
                {"speed_topic": "talon_1_speed"},
                {"info_topic": "talon_1_info"},
                {"position_topic": "talon_1_position"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 50},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can0"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='talon',
            name='Talon2',
            executable='talon_node',
            parameters=[
                {"motor_number": 2},
                {"diagnostics_port": 56714},
                {"invert_motor": True},
                {"speed_topic": "talon_2_speed"},
                {"info_topic": "talon_2_info"},
                {"position_topic": "talon_2_position"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 51},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can0"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
    ]
)
