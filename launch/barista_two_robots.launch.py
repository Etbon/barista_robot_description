import os
from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction, IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def generate_launch_description():
    # Get the absolute path
    pkg_name = 'barista_robot_description'
    src_share = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(src_share, 'xacro', 'barista_robot_model.urdf.xacro')

    # Set Gazebo model paths (critical for meshes)
    install_dir = get_package_prefix(pkg_name)
    gazebo_models_path = os.path.join(src_share, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += f":{gazebo_models_path}:{install_dir}/share"
    else:
        os.environ['GAZEBO_MODEL_PATH'] = f"{gazebo_models_path}:{install_dir}/share"

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                        'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(src_share, 'worlds', 'barista_robot_empty.world')
        }.items()
    )
    
    # Publish world → map once (static)
    map_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0','0','0','0','0','0','world','map']
    )

    # Robot Spawning Helper
    def spawn_robot(ns: str, x: float, y: float, color: str):
        robot_description = Command([
            FindExecutable(name='xacro'), ' ', xacro_file,
            ' robot_name:=', ns,
            ' include_laser:=true',
            ' include_wheels:=true',
            ' color:=', color,
            ' plugin_ns:=' + ns
        ])
        return GroupAction([
            PushRosNamespace(ns),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{
                    'robot_description': robot_description,
                    'frame_prefix': f'{ns}/',
                    'use_sim_time': True
                }],
                output='screen'
            ),
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-topic', f'/{ns}/robot_description',  # Namespaced topic
                    '-entity', ns,
                    '-x', str(x), '-y', str(y), '-z', '0.1',
                    '-robot_namespace', ns
                ],
                output='screen'
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0',  # xyz rpy
                    'map', f'{ns}/odom'],
                parameters=[{'use_sim_time': True}],
                name=f'{ns}_map_to_odom'
            ),
        ])

    # Spawn robots with staggered delays
    rick = TimerAction(period=3.0, actions=[spawn_robot('rick', -1.0, 0.0, 'Red')])
    morty = TimerAction(period=5.0, actions=[spawn_robot('morty', 1.0, 0.0, 'Blue')])

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(src_share, 'rviz', 'barista_two_robots.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        map_tf,
        rick,
        morty,
        rviz_node
    ])