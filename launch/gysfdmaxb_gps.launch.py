#import launch
#import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0', description=''),
        DeclareLaunchArgument('frame_id', default_value='gps_link', description=''),
        DeclareLaunchArgument('rate', default_value='6', description=''),
        DeclareLaunchArgument('topicName', default_value='fix', description=''),
        Node(
            package='gysfdmaxb_gps',
            executable='gysfdmaxb_gps_node',
            output="screen",
            #emulate_tty=True,
            parameters=[
                        {"device": LaunchConfiguration('device') ,
                         "frame_id": LaunchConfiguration('frame_id'),
                         "rate": LaunchConfiguration('rate'),
                         "topicName": LaunchConfiguration('topicName'),
                         }
            ]
        )
    ])

'''
#https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cpp_parameters",
            executable="minimal_param_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"my_parameter": "earth"}
            ]
        )
    ])
'''