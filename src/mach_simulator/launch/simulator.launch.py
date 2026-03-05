import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('mach_simulator')
    description_pkg = get_package_share_directory('mach_description')
    stereo_pkg = get_package_share_directory('stereo_image_proc')

    urdf_path = os.path.join(
        description_pkg,
        'urdf',
        'bfmc_car.urdf.xacro'
    )

    world_path = os.path.join(
        pkg_share,
        'worlds',
        'full-track.world'
    )

    rviz_config_path = os.path.join(
        description_pkg,
        'rviz',
        'bfmc_sim.rviz'
    )

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(pkg_share, 'models')
    )

    # ---- Robot State Publisher ----
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True
        }],
        output='screen'
    )

    # ---- Gazebo ----
    gazebo = ExecuteProcess(
        cmd=[
            'bash', '-lc',
            f'source /usr/share/gazebo/setup.sh && '
            f'gazebo --verbose "{world_path}" '
            f'-s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
        ],
        output='screen'
    )


    # ---- Spawn Robot ----
    spawn_car = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'bfmc_car',
            '-topic', 'robot_description',
            '-x', '1.67',
            '-y', '-6.57',
            '-z', '0.1',
            '-Y', '-0.08'
        ],
        output='screen'
    )

    # ---- Controllers ----
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    ackermann_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_controller'],
        output='screen'
    )

    ackermann_to_twist = Node(
        package='mach_simulator',
        executable='ackermann_to_twist',
        name='ackermann_to_twist',
        parameters=[{
            'wheelbase': 0.255,
            'input_topic': '/drive_cmd',
            'output_topic': '/ackermann_controller/reference',
        }],
        output='screen'
    )

    # ---- Stereo Node ----
    stereo_vision = GroupAction(
        actions=[
            PushRosNamespace('camera'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(stereo_pkg, 'launch', 'stereo_image_proc.launch.py')
                ),
                launch_arguments={
                    'approximate_sync': 'True',
                    'left_namespace': 'left',
                    'right_namespace': 'right',
                    'disparity_range': '64',
                    'speckle_size': '0',
                    'texture_threshold': '2000',
                }.items()
            )
        ]
    )

    # ---- RViz Node ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        robot_state_publisher,
        gazebo,
        spawn_car,
        joint_state_broadcaster,
        ackermann_controller,
        ackermann_to_twist,
        # stereo_vision,
        # rviz
    ])