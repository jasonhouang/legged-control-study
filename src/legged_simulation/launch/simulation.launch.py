import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # Process xacro
    robot_description_path = os.path.join(
        FindPackageShare('legged_a1_description').find('legged_a1_description'),
        'urdf', 'robot.xacro'
    )
    robot_description_config = xacro.process_file(robot_description_path)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Gazebo (with GUI)
    gz_sim = ExecuteProcess(
        cmd=['bash', '-c', 'export DISPLAY=:0 && ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=\"-v4 empty.sdf\"'],
        output='screen',
        additional_env={'DISPLAY': ':0'}
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'a1',
            '-topic', 'robot_description',
            '-z', '0.25',  # 降低初始高度，减少落地冲击
        ],
        output='screen',
    )

    # Load controllers (inactive first, will activate after spawn)
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--inactive'],
        output='screen',
    )

    effort_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_group_effort_controller', '--inactive'],
        output='screen',
    )

    # Activate controllers after spawn (wait for simulation to stabilize)
    activate_controllers = ExecuteProcess(
        cmd=['bash', '-c', '''
            sleep 5
            ros2 control switch_controllers --activate joint_state_broadcaster joint_group_effort_controller
        '''],
        output='screen',
    )

    # Stand controller node
    stand_controller_node = Node(
        package='legged_wbc_ros2',
        executable='stand_controller',
        name='stand_controller',
        output='screen',
        parameters=[{
            'control_frequency': 200.0,
            'kp': 1.0,
            'kd': 20.0,
            'robot_mass': 13.5,
            'ramp_time': 3.0,
        }],
    )

    # Bridge: /clock and odometry from gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/model/a1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen',
    )

    return LaunchDescription([
        bridge,
        gz_sim,
        robot_state_publisher_node,
        spawn_robot,
        # Step 1: Load controllers (inactive) after robot spawned
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[effort_controller_spawner],
            )
        ),
        # Step 2: Activate controllers after loading
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=effort_controller_spawner,
                on_exit=[activate_controllers],
            )
        ),
        # Step 3: Start stand controller after controllers active
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=activate_controllers,
                on_exit=[stand_controller_node],
            )
        ),
    ])
