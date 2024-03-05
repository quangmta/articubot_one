import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('realsense2_camera'),'launch','rs_launch.py'
            )]), launch_arguments={'pointcloud.enable':'true',
                                   'pointcloud.ordered_pc': 'true',
                                   'depth_module.profile': '424x240x6',
<<<<<<< HEAD
                                   'rgb_camera.profile': '320x240x6',
=======
                                   'rgb_camera.profile': '320x240x6',                                   
>>>>>>> 4bce3494d43616f99c7cae361ae9fd675ab95f13
                                   'align_depth.enable':'true'}.items()
                                   
        )
    ])