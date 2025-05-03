import os
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

import xacro

# this is the function launch  system will look for
def generate_launch_description():

    # ####### DATA INPUT ##########
    # urdf_file = 'robot.urdf'
    # #xacro_file = "urdfbot.xacro"
    package_description = "robot_desc"
    # robot_description_package = launch_ros.substitutions.FindPackageShare(package='robot_desc').find('robot_desc')

    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(package_description), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    #robot_desc_path = os.path.join(get_package_share_directory(package_description), "robot", urdf_file)

    urdf_file_name = 'robot.urdf'
    urdf = os.path.join(
        get_package_share_directory(package_description),
        'robot',
        urdf_file_name)
    
    print("Found UDRF")
    
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()


    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        # parameters=[{'use_sim_time': True, 'robot_description': launch_ros.parameter_descriptions.ParameterValue( launch_ros.substitutions.Command(['xacro ', os.path.join(robot_description_package, 'robot/robot.udrf')]), value_type=str) }],
        output="screen"
    )


    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node,
        ]
    )
