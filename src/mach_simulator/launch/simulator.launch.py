import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    pkg_share = get_package_share_directory('mach_simulator')
    description_pkg = get_package_share_directory('mach_description')
    gazebo_assets_share = get_package_share_directory('gazebo_assets')

    urdf_path = os.path.join(
        description_pkg,
        'urdf',
        'bfmc_car.urdf.xacro'
    )

    world_path = os.path.join(
        gazebo_assets_share,
        'worlds',
        'world.world'
    )

    rviz_config_path = os.path.join(
        pkg_share,
        'config',
        'bfmc_sim.rviz'
    )

    set_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(gazebo_assets_share, 'models')
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
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
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

    cmd_vel_relay = Node(
        package='topic_tools',
        executable='relay',
        arguments=[
            '/cmd_vel',
            '/ackermann_controller/reference_unstamped'
        ],
        output='screen'
    )

    # ---- RViz Node ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_path],
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
        cmd_vel_relay,
        rviz
    ])