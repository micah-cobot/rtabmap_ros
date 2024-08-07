# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true
#   $ ros2 param set /camera/camera depth_module.emitter_enabled 0
#
#   $ ros2 launch rtabmap_examples realsense_d435i_stereo.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    rs_node_name = 'rs_D457_front'
    rs_node_namespace = ''

    rs_global_prefix = f'/{rs_node_name}' if not rs_node_namespace else f'/{rs_node_namespace}/{rs_node_name}'

    parameters=[{   
          'frame_id':'camera_link',
          'subscribe_stereo':True,
          'subscribe_odom_info':False,
          'wait_imu_to_init':True,
          'approx_sync': True,
          'approx_sync_max_interval': 0.01,
          'sync_queue_size': 100,
          'topic_queue_size': 1000,
          'wait_for_transform': 1.0,
          'tf_tolerance': 1.0,
        }]

    # remappings=[
    #       ('imu', '/imu/data'),
    #       ('left/image_rect', f'{rs_global_prefix}/infra1/image_rect_raw'),
    #       ('left/camera_info', f'{rs_global_prefix}/infra1/calibrated_camera_info'),
    #       ('right/image_rect', f'{rs_global_prefix}/infra2/image_rect_raw'),
    #       ('right/camera_info', f'{rs_global_prefix}/infra2/calibrated_camera_info')]
    remappings=[
          ('imu', '/imu/data'),
          ('left/image_rect', f'{rs_global_prefix}/infra1/image_rect_raw'),
          ('left/camera_info', f'{rs_global_prefix}/infra1/camera_info'),
          ('right/image_rect', f'{rs_global_prefix}/infra2/image_rect_raw'),
          ('right/camera_info', f'{rs_global_prefix}/infra2/camera_info')]

    return LaunchDescription([

        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='stereo_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

        # Node(
        #     package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        #     parameters=parameters,
        #     remappings=remappings),
                
        # Compute quaternion of the IMU
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{'use_mag': False, 
                         'world_frame':'enu', 
                         'publish_tf':False}],
            remappings=[('imu/data_raw', f'{rs_global_prefix}/imu')]),
        
        # # The IMU frame is missing in TF tree, add it:
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'camera_gyro_optical_frame', 'camera_imu_optical_frame']),
    ])
