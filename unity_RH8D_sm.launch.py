from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define launch arguments
    port_arg = DeclareLaunchArgument('port', default_value='9090', description='Port number for ROS communication')

    # Define paths
    pkg = get_package_share_directory('TFG')
    rosbridge_server_launch_file = PathJoinSubstitution(
        [pkg, 'launch', 'ros_sharp_communication.launch.py'])

    # Include ROS Sharp Communication launch file
    ros_sharp_communication_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbridge_server_launch_file]),
                launch_arguments=[('port', LaunchConfiguration('port'))]
        )
    
    
    return LaunchDescription([
        port_arg,
        ros_sharp_communication_launch,
        Node(
            package='TFG',
            executable='state_machine_mod',
            emulate_tty=True,
            output='screen'
        )
    ])
