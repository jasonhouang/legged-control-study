import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 直接定义 URDF
    robot_description = '''<?xml version="1.0"?>
<robot name="test_joint">
  <link name="world"/>
  
  <link name="base">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <link name="arm">
    <visual>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0.25" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  
  <joint name="world_to_base" type="fixed">
    <parent link="world"/>
    <child link="base"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
  </joint>
  
  <joint name="test_joint" type="revolute">
    <parent link="base"/>
    <child link="arm"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
  </joint>
  
  <gazebo>
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>test_joint</joint_name>
      <position>0.0</position>
      <p_gain>100.0</p_gain>
      <i_gain>0.0</i_gain>
      <d_gain>10.0</d_gain>
    </plugin>
  </gazebo>
</robot>'''
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': 'empty.sdf'}.items(),
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'test_joint',
            '-topic', 'robot_description',
            '-z', '1.0',
        ],
        output='screen',
    )
    
    # 延迟启动关节控制命令
    joint_command = ExecuteProcess(
        cmd=['bash', '-c', '''
            sleep 5
            source /opt/ros/jazzy/setup.bash
            echo "发送位置命令: 1.0 rad"
            ros2 topic pub --once /model/test_joint/joint/test_joint/cmd_pos std_msgs/msg/Float64 "{data: 1.0}"
            sleep 2
            echo "发送位置命令: -1.0 rad"
            ros2 topic pub --once /model/test_joint/joint/test_joint/cmd_pos std_msgs/msg/Float64 "{data: -1.0}"
        '''],
        output='screen',
    )
    
    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        joint_command,
    ])
