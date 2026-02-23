import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'my_rover'
    pkg_path = get_package_share_directory(package_name)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_path, 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Por esto:
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
        )]),
        launch_arguments={'gz_args': '-r /opt/ros/jazzy/share/turtlebot3_gazebo/worlds/turtlebot3_world.world'}.items()
    )



    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'rover','-x', '-1.5', '-y', '0.0', '-z', '0.01'],
        output='screen'
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--unload-on-kill'],
    )
    

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--unload-on-kill'],
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=os.path.join(pkg_path, '..', '..')  # apunta a install/my_rover/share
        ),
        rsp,
        gazebo,
        spawn_entity,
        bridge,
        diff_drive_spawner,
        joint_broad_spawner,
    ])
