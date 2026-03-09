from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        # Static TF
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0.6", "0.1", "0.2",
                "0", "0", "0",
                "base",
                "surface_frame"
            ]
        ),

        # Zigzag node
        Node(
            package="zigzag_toolpath",
            executable="zigzag_tf_node",
            parameters=[
                {"alpha": 0.3}
            ]
        )
    ])