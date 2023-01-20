from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # config = os.path.join(
    # get_package_share_directory('sp_module_client'),
    # 'config',
    # 'module_params.yaml'
    # )
    sealer_port = LaunchConfiguration("sealer_port")
    peeler_port = LaunchConfiguration("peeler_port")

    declare_use_sealer_port_cmd = DeclareLaunchArgument(
        name='sealer_port',
        default_value="/dev/ttyUSB1",
        description='Flag to accept sealer port number'
        )
    
    declare_use_peeler_port_cmd = DeclareLaunchArgument(
        name = "peeler_port",
        default_value= "/dev/ttyUSB0",
        description= "Flag to accept peeler port"
        )

    peeler=Node(
        package='sp_module_client',
        namespace = 'std_ns',
        executable='peeler_client',
        name='PeelerNode',
        parameters = [{peeler_port:"peeler_port"}]
    )

    sealer=Node(
        package='sp_module_client',
        namespace = 'std_ns',
        executable='sealer_client',
        name='SealerNode',
        parameters = [{sealer_port:"sealer_port"}]
    )

    ld.add_action(declare_use_sealer_port_cmd)
    ld.add_action(declare_use_peeler_port_cmd)
    ld.add_action(peeler)
    ld.add_action(sealer)

    return ld
