from launch import LaunchDescription
from launch_ros.actions import Node
import os
from glob import glob
import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

pkg_name = 'wheelbot_control'

def generate_launch_description():
    ld = LaunchDescription(
    )
          
    wheelbot_control = Node(
        package             = pkg_name,
        executable          = 'wheelbot_control_node',
        name                = 'wheelbot_control_node',
        #output             = 'screen',
        #respawn             = 'true',#：該当ノードが終了した場合に、再起動するようにする。
    )

    joy = Node(
        package             = 'joy',
        executable          = 'joy_node',
        name                = 'joy_node',
        #output             = 'screen',
        #respawn             = 'true',#：該当ノードが終了した場合に、再起動するようにする。
    )

    # joy_translate = Node(
    #     package             = 'joy_translate',
    #     executable          = 'joy_translate_node',
    #     name                = 'joy_translate_node',
    #     #output             = 'screen',
    #     #respawn             = 'true',#：該当ノードが終了した場合に、再起動するようにする。
    # )

    
    teleop_twist_joy = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("teleop_twist_joy") + "/launch/teleop-launch.py"]),
    )

    ld.add_action(wheelbot_control)
    ld.add_action(joy)
    #ld.add_action(joy_translate)
    ld.add_action(teleop_twist_joy )
    return ld