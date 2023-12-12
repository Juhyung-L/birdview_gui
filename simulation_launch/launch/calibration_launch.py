import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_launch')
    launch_file_dir = os.path.join(pkg_share, 'launch')
    script_file_dir = os.path.join(pkg_share, 'scripts')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # parameters for generate_moving_checkerboard.py
    length = LaunchConfiguration('length', default=5)
    width = LaunchConfiguration('width', default=5)
    height = LaunchConfiguration('height', default=10)
    x_off = LaunchConfiguration('x_off', default=3)
    z_off = LaunchConfiguration('z_off', default=7)
    trajectory_length = LaunchConfiguration('trajectory_length', default=60)
    num_pts = LaunchConfiguration('num_pts', default=60)
    time_to_complete = LaunchConfiguration('time_to_complete', default=60)
    rows = LaunchConfiguration('rows', default=10)
    cols = LaunchConfiguration('cols', default=7)
    sq_size = LaunchConfiguration('sq_size', default=0.3)
    world_file = LaunchConfiguration('world_file', default='/home/dev_ws/src/simulation_launch/worlds/calibration.world')
    robot_model_file = LaunchConfiguration('robot_model_file', default='/home/dev_ws/src/simulation_launch/urdf/camera.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    lockstep = LaunchConfiguration('lockstep', default='True')

    declare_length = DeclareLaunchArgument(
        name='length',
        default_value=length,
        description='Length of the elliptical base of the trajectory'
    )
    declare_width = DeclareLaunchArgument(
        name='width',
        default_value=width,
        description='Width of the elliptical base of the trajectory'
    )
    declare_height = DeclareLaunchArgument(
        name='height',
        default_value=height,
        description='Height of the helical trajectory'
    )
    declare_x_off = DeclareLaunchArgument(
        name='x_off',
        default_value=x_off,
        description="Offset in the +x direction of the checkerboard's starting position"
    )
    declare_z_off = DeclareLaunchArgument(
        name='z_off',
        default_value=z_off,
        description="Offset in the +z direction of the checkerboard's starting position"
    )
    declare_trajectory_length = DeclareLaunchArgument(
        name='trajectory_length',
        default_value=trajectory_length,
        description='Proportional to the total distance of the trajectory'
    )
    declare_num_pts = DeclareLaunchArgument(
        name='num_pts',
        default_value=num_pts,
        description='Number of key points in the trajectory'
    )
    declare_time_to_complete = DeclareLaunchArgument(
        name='time_to_complete',
        default_value=time_to_complete,
        description='Total time to complete the trajectory'
    )
    declare_rows = DeclareLaunchArgument(
        name='rows',
        default_value=rows,
        description='Number of rows in the checkerboard pattern'
    )
    declare_cols = DeclareLaunchArgument(
        name='cols',
        default_value=cols,
        description='Number of columns in the checkerboard pattern'
    )
    declare_sq_size = DeclareLaunchArgument(
        name='sq_size',
        default_value=sq_size,
        description='Size of each square in the checkerboard'
    )
    declare_world_file = DeclareLaunchArgument(
        name='world_file',
        default_value=world_file,
        description='Path to the Gazebo world file'
    )
    declare_robot_model_file = DeclareLaunchArgument(
        name='robot_model_file',
        default_value=robot_model_file,
        description='Path to the robot urdf file'
    )
    delcare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value=use_sim_time,
        description='Use Gazebo time if true'
    )
    declare_lockstep = DeclareLaunchArgument(
        name='lockstep',
        default_value=lockstep,
        description='Run Gazebo with lockstep if true'
    )

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'lockstep': lockstep,
            'use_sim_time': use_sim_time
        }.items()
    )

    gzclient_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    generate_world_file = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(script_file_dir, 'generate_moving_checkerboard.py'),
            length,
            width,
            height,
            x_off,
            z_off,
            trajectory_length,
            num_pts,
            time_to_complete,
            rows,
            cols,
            sq_size,
            world_file
        ],
        output='screen'
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_model_file': robot_model_file,
            'x_pose': '0.0',
            'y_pose': '0.0',
            'z_pose': z_off
        }.items()
    )

    return LaunchDescription([
        declare_length,
        declare_width,
        declare_height,
        declare_x_off,
        declare_z_off,
        declare_trajectory_length,
        declare_num_pts,
        declare_time_to_complete,
        declare_rows,
        declare_cols,
        declare_sq_size,
        declare_world_file,
        declare_robot_model_file,
        delcare_use_sim_time,
        declare_lockstep,

        gzserver_launch,
        gzclient_launch,
        generate_world_file,
        spawn_robot_launch
    ])