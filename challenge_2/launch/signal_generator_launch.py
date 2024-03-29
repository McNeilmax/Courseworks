import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    config = os.path.join(
            get_package_share_directory('challenge_2'),
                                        'config',
                                        'signal_params.yaml')

    signal_generator_node = Node(
            package='challenge_2',
            executable='signal_generator',
            name = 'signal_generator_node',
            output = 'screen',
            parameters = [config]
    )

    signal_reconstructed_node = Node(
            package='challenge_2',
            executable='signal_reconstructed',
            name = 'signal_reconstructed_node',
            output = 'screen',
            )

    l_d = LaunchDescription([signal_generator_node, signal_reconstructed_node])

    return l_d
