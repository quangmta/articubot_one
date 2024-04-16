import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'),'launch','rs_launch.py'
            )]), launch_arguments={'pointcloud.enable':'false',
                                   'enable_gyro':'true',
                                   'enable_accel':'true',
                                   'unite_imu_method':'2',
                                   'enable_motion_correction':'true',
                                #    'enable_infra1':'true',
                                #    'enable_infra2':'true',
                                #    'enable_color':'true',
                                #    'pointcloud.ordered_pc': 'true',
                                   'depth_module.profile': '424x240x6',
                                   'rgb_camera.profile': '320x240x6',                                   
                                   'align_depth.enable':'true'}.items()
                                   
        )
        ,
        Node(
                package="imu_filter_madgwick",
                executable="imu_filter_madgwick_node",
                parameters=[{'use_mag': False}],
                remappings=[('/imu/data_raw','/camera/imu')]
            )
    ])