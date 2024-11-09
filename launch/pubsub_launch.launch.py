from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    msg_content_arg = DeclareLaunchArgument(
        "message_content", default_value="My name is Datta Lohith Gannavarapu.",
        description="Initial message content for the publisher")

    publisher_node = Node(
        package="beginner_tutorials",
        executable="talker",
        parameters=[{"message_content": LaunchConfiguration("message_content")}]
    )

    subscriber_node = Node(
        package="beginner_tutorials",
        executable="listener",
    )

    return LaunchDescription([
        msg_content_arg,
        publisher_node,
        subscriber_node
    ])
