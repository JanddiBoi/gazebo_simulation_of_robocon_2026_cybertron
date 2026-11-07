import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # -------------------------------------------------------------------------
    # 1. Package Paths
    # -------------------------------------------------------------------------
    pkg_share = get_package_share_directory('nav2_test')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    default_xacro_path = os.path.join(pkg_share, 'model', 'mecanum_robot.urdf.xacro')
    default_world_path = os.path.join(pkg_share, 'world', 'robocon2027_world.sdf')
    default_rviz_path  = os.path.join(pkg_share, 'rviz', 'robot_view.rviz')

    # -------------------------------------------------------------------------
    # 2. Launch Configuration Variables & Arguments
    # -------------------------------------------------------------------------
    use_sim_time = LaunchConfiguration('use_sim_time')
    world        = LaunchConfiguration('world')
    x_pose       = LaunchConfiguration('x')
    y_pose       = LaunchConfiguration('y')
    z_pose       = LaunchConfiguration('z')
    yaw_pose     = LaunchConfiguration('R')
    rviz_config  = LaunchConfiguration('rvizconfig')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world', default_value=default_world_path, description='Path to SDF world file'
    )

    declare_x_cmd = DeclareLaunchArgument('x', default_value='0.0')
    declare_y_cmd = DeclareLaunchArgument('y', default_value='0.0')
    declare_z_cmd = DeclareLaunchArgument('z', default_value='0.1')
    declare_yaw_cmd = DeclareLaunchArgument('R', default_value='0.0')

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz_path, description='Path to RViz config file'
    )

    # -------------------------------------------------------------------------
    # 3. Robot State Publisher (Xacro -> URDF)
    # -------------------------------------------------------------------------
    robot_description_content = ParameterValue(
        Command(['xacro ', default_xacro_path]),
        value_type=str
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    # -------------------------------------------------------------------------
    # 4. Launch Gazebo Harmonic
    # -------------------------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # Build gz_args by concatenating substitutions directly. The previous
        # version used Command(['echo "-r "', world]), which spawns a real
        # subshell and captures its stdout INCLUDING the trailing newline that
        # `echo` appends. That stray newline split the eventual `gz sim ...`
        # command across two lines, so the `--force-version 8` argument
        # appended later by gz_sim.launch.py was executed as its own (bogus)
        # shell command -> "exit code 127 / --force-version: not found".
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # -------------------------------------------------------------------------
    # 5. Spawn URDF Robot
    # -------------------------------------------------------------------------
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_mecanum_robot',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mecanum_robot',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw_pose
        ],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 6. ros_gz_bridge (Parameter Bridge for Topics & TF Transforms)
    # -------------------------------------------------------------------------
    bridge_params = [
        # Motors / Odom / Clock
        '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

        # Joint states -> lets robot_state_publisher fill in TF for the
        # wheel joints and camera_slider_joint (see JointStatePublisher
        # plugin added in the xacro).
        '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

        # LaserScan LiDAR Data
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

        # RGBD Camera: gz-sim publishes rgbd_camera sensors under
        # <topic>/image, <topic>/depth_image, <topic>/camera_info (NOT
        # "image_raw" -- that ROS-side topic name was never actually
        # produced by Gazebo, hence the topic existing with zero data).
        '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
        '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

        # Prismatic Camera Slider Position Command
        '/camera_height_cmd@std_msgs/msg/Float64]gz.msgs.Double',
    ]

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=bridge_params,
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # -------------------------------------------------------------------------
    # 7. RViz2 Node
    # -------------------------------------------------------------------------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # -------------------------------------------------------------------------
    # 8. Launch Description Assembly
    # -------------------------------------------------------------------------
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_cmd,
        declare_x_cmd,
        declare_y_cmd,
        declare_z_cmd,
        declare_yaw_cmd,
        declare_rviz_config_cmd,
        node_robot_state_publisher,
        gz_sim,
        gz_spawn_entity,
        ros_gz_bridge,
        rviz_node
    ])