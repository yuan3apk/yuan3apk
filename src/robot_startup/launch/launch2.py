from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
  
    raw_image2=Node(
        package="test2_cpp",
        executable="raw_image2"
    )
    process_image2=Node(
        package="test2_cpp",
        executable="process"
    )
    launch_description=LaunchDescription([

        raw_image2,
        process_image2,
    ])
    return launch_description
