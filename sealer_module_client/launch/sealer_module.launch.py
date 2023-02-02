from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()


    sealer_port = LaunchConfiguration("sealer_port")

    declare_use_sealer_port_cmd = DeclareLaunchArgument(
        name='sealer_port',
        default_value="/dev/ttyUSB1",
        description='Flag to accept sealer port number'
        )
    

    sealer=Node(
        package='sealer_module_client',
        namespace = 'std_ns',
        executable='sealer_client',
        name='SealerNode',
        parameters = [{"sealer_port":sealer_port}],            
        emulate_tty=True

    )

    ld.add_action(declare_use_sealer_port_cmd)
    ld.add_action(sealer)

    return ld
