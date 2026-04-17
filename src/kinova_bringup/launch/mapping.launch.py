from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters = [{
        'frame_id': 'realsense_link',
        'odom_frame_id': 'base_link',
        'subscribe_depth': True,
        'subscribe_odom': False,
        'subscribe_rgb': True,
        'approx_sync': True,
        'wait_for_transform': 0.2,
        'database_path': '~/.ros/rtabmap.db',
    }]

    remappings = [
        ('rgb/image', 'camera/realsense/color/image_raw'),
        ('rgb/camera_info', 'camera/realsense/color/camera_info'),
        ('depth/image', 'camera/realsense/aligned_depth_to_color/image_raw')
    ]

    rtabmap_node = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=parameters,
        remappings=remappings,
        arguments=['-d'] # Clear database on start
    )

    return LaunchDescription([
        rtabmap_node
    ])
