from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_model_file = LaunchConfiguration('robot_model_file')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    declare_robot_model_file = DeclareLaunchArgument(
        name='robot_model_file',
        default_value='',
        description='Path to robot model file'
    )
    declare_x_position = DeclareLaunchArgument(
        name='x_pose', 
        default_value='0.0',
        description='Starting x position of the robot'
    )
    declare_y_position = DeclareLaunchArgument(
        name='y_pose', 
        default_value='0.0',
        description='Starting y position of the robot'
    )
    declare_z_position = DeclareLaunchArgument(
        name='z_pose',
        default_value='0.0',
        description='Starting z position of the robot'
    )

    start_gazebo_ros_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', '',
            '-file', robot_model_file,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_x_position,
        declare_y_position,
        declare_z_position,
        declare_robot_model_file,
        start_gazebo_ros_spawner
    ])