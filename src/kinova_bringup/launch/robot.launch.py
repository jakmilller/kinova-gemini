from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# this launches the robot description, realSense, and custom controller
def generate_launch_description():
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='IP address of the robot'
    )

    username_arg = DeclareLaunchArgument(
        'username',
        default_value='admin',
        description='Username for robot connection'
    )
    
    password_arg = DeclareLaunchArgument(
        'password',
        default_value='admin',
        description='Password for robot connection'
    )
    
    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value='gen3',
        description='Type/series of robot'
    )
    
    dof_arg = DeclareLaunchArgument(
        'dof',
        default_value='7',
        description="Robot's degrees of freedom"
    )
    
    gripper_arg = DeclareLaunchArgument(
        'gripper',
        default_value='robotiq_2f_140',
        description='Gripper type'
    )

    # Boolean feature toggles (pass e.g. rviz:=false voice:=true on the command line)
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    voice_arg = DeclareLaunchArgument(
        'voice',
        default_value='true',
        description='Launch the push-to-talk voice interface node'
    )

    arduino_arg = DeclareLaunchArgument(
        'arduino',
        default_value='true',
        description='Launch the physical Arduino button bridge node'
    )

    # robot description direct from pdf
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("kortex_description"), "robots", "kinova.urdf.xacro"]
            ),
            " ",
            "robot_ip:=", LaunchConfiguration('robot_ip'),
            " ",
            "name:=kinova",
            " ",
            "arm:=", LaunchConfiguration('robot_type'),
            " ",
            "gripper:=", LaunchConfiguration('gripper'),
            " ",
            "dof:=", LaunchConfiguration('dof'),
            " ",
            "vision:=true",
            " ",
            "use_fake_hardware:=false",
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    # publish transform and description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    # we have mounted RealSense camera, create static transform
    # should technically adjust urdf
    realsense_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '-0.01',  # x offset (all offsets calculated from mounted RealSense)
            '0.003',    # y offset
            '0.031',   # z offset
            '-1.5708',    # roll
            '-1.5708',    # pitch
            '3.14159',    # yaw
            'camera_link',  # kinova camera frame
            'realsense_link'  # external RealSense frame
        ],
        name='realsense_transform'
    )

# the first one makes sure that the pointclouds are visualized right, this one is for inspect_scene
    physical_realsense_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '-0.02',  # x offset (all offsets calculated from mounted RealSense)
            '-0.003',    # y offset
            '0.031',   # z offset
            '0',    # roll
            '0',    # pitch
            '0',    # yaw
            'camera_link',  # kinova camera frame
            'physical_realsense_link'  # external RealSense frame
        ],
        name='physical_realsense_transform'
    )

    # RViz for visualization
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("kinova_bringup"), "config", "robot.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        output="screen",
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Push-to-talk voice input nodes (packaged in gemini_live_robotics). Off by default so a
    # plain `robot.launch.py` brings up only hardware + visualization; opt in with voice:=true.
    voice_interface_node = Node(
        package='gemini_live_robotics',
        executable='voice_interface',
        name='voice_interface_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('voice')),
    )

    arduino_trigger_node = Node(
        package='gemini_live_robotics',
        executable='arduino_trigger',
        name='arduino_trigger_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('arduino')),
    )

    # forwarding=False stops this launch's own arguments (robot_ip, username, dof, gripper, ...)
    # from being forwarded into the RealSense launch, which would otherwise reject each of them
    # as an unsupported parameter and flood the console. RealSense still receives the arguments
    # explicitly passed below.
    realsense_launch = GroupAction(
        scoped=True,
        forwarding=False,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    ])
                ]),
                launch_arguments={
                    'align_depth.enable': 'true',
                    'pointcloud.enable': 'true',
                    'camera_name': 'realsense',
                }.items()
            )
        ]
    )

    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            ])
        ])
    )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen'
    )

    controller_node = Node(
        package='kortex_controller',
        executable='controller',
        name='kinova_controller',
        output='screen',
        parameters=[{
            'robot_ip': LaunchConfiguration('robot_ip'), # Changed from ip_address
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
        }]
    )

    return LaunchDescription([
        robot_ip_arg,
        username_arg,
        password_arg,
        robot_type_arg,
        dof_arg,
        gripper_arg,
        rviz_arg,
        voice_arg,
        arduino_arg,
        controller_node,
        robot_state_publisher,
        realsense_transform,
        physical_realsense_transform,
        realsense_launch,
        rosbridge_launch,
        web_video_server_node,
        rviz_node,
        voice_interface_node,
        arduino_trigger_node,
    ])