import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    ip_address_launch_arg = launch.actions.DeclareLaunchArgument(
        'ip_address', default_value='192.168.194.95'
    )
    frame_id_launch_arg = launch.actions.DeclareLaunchArgument(
        'frame_id', default_value='odom'
    )

    dvl_a50 = launch_ros.actions.Node(
        package='dvl_a50',
        executable='dvl_a50_sensor',
        parameters=[{'dvl_ip_address': launch.substitutions.LaunchConfiguration('ip_address')}],
        output='screen'
    )

    position_to_path_node = launch_ros.actions.Node(
        package='dvl_a50',
        executable='position_to_path_node.py',
        name='position_to_path_node',
        output='screen',
        parameters=[{'frame_id': launch.substitutions.LaunchConfiguration('frame_id')}]
    )

    static_transform_publisher_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    return launch.LaunchDescription([
        ip_address_launch_arg,
        frame_id_launch_arg,
        dvl_a50,
        position_to_path_node,
        static_transform_publisher_node,
    ])


       
