from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
    get_package_share_directory('sp_module_client'),
    'config',
    'module_params.yaml'
    )
        
    peeler=Node(
        package='sp_module_client',
        namespace='sp_module',
        executable='peelerNode',
        name='peelerNode',
        parameters = [config]
    )

    sealer=Node(
        package='sp_module_client',
        namespace='sp_module',
        executable='sealerNode',
        name='sealerNode',
        parameters = [config]
    )

    ld.add_action(peeler)
    ld.add_action(sealer)

    return ld