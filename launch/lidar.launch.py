import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([

        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('urg_node2'),'launch','urg_node2.launch.py'
                )]), launch_arguments={'frame_id':'laser_frame',
                                       'serial_port':'/dev/ttyACM0', 
                                       'angle_min': '-2.0944',
                                       'angle_max': '2.0944'}.items()
        )
    ])
