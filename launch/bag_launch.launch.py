from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    msg_content_arg = DeclareLaunchArgument(
        "message_content", default_value="My name is Datta Lohith Gannavarapu.",
        description="Initial message content for the publisher")

    record_bag_arg = DeclareLaunchArgument(
        "record_bag", default_value="True", choices=["True", "False"],
        description="Whether to record a bag file")

    stop_args = DeclareLaunchArgument(
        "stop", default_value="False", choices=["True", "False"],
        description="Whether to stop the launch file after 15 seconds")
        
    # Publisher node with the message content parameter
    publisher_node = Node(
        package="beginner_tutorials",
        executable="talker",
        name="talker",
        parameters=[{
            "message_content": LaunchConfiguration("message_content")
        }]
    )

    # Subscriber node
    subscriber_node = Node(
        package="beginner_tutorials",
        executable="listener",
        name="listener"
    )

    # Recorder node for recording bag file if `record_bag` is True and delete the existing bag files if any and record the new bag file
    recorder_node = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration("record_bag")),
        cmd=[
            "bash", "-c",
            "if [ -d 'results/bag_list/' ]; then rm -rf results/bag_list/; fi; ros2 bag record -o results/bag_list/ -a"
        ],
    )

    # Timer action to stop the launch file after 15 seconds if `stop_args` is True
    stop_action = TimerAction(
        period=15.0,  # 15 seconds
        actions=[Shutdown()],
        condition=IfCondition(LaunchConfiguration("stop"))
    )

    return LaunchDescription([
        msg_content_arg,
        record_bag_arg,
        stop_args,
        publisher_node,
        subscriber_node,
        recorder_node,
        stop_action
    ])
