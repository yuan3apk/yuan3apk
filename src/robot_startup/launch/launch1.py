from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
  
    raw_image=Node(
        package="test_cpp",
        executable="raw_image"
    )
    process_image1=Node(
        package="test_cpp",
        executable="process_image1"
    )
    process_image2=Node(
        package="test_cpp",
        executable="process_image2"
    )
    image=Node(
        package="test_cpp",
        executable="image"
    )
    launch_description=LaunchDescription([

        raw_image,
        process_image1,
        process_image2,
        image
    ])
    return launch_description
