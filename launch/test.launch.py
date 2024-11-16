from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_catch_ros2 import Catch2IntegrationTestNode, Catch2LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the argument for test duration
    test_duration_arg = DeclareLaunchArgument(
        name='test_duration',
        default_value='2.0',
        description='Max length of test in seconds.'
    )

    # Node for the integration test using the TestNode class
    test_talker_node = Catch2IntegrationTestNode(
        package='beginner_tutorials',
        executable='test_talker',
        name='test_talker',
        parameters=[{'test_duration': LaunchConfiguration('test_duration')}],
    )
    
    talker_node = Catch2IntegrationTestNode(
        package='beginner_tutorials',
        executable='talker',
        name='talker',
    )

    return Catch2LaunchDescription([
        test_duration_arg,
        talker_node,
        test_talker_node,
    ])
