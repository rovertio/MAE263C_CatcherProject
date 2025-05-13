import os
import launch_ros.descriptions

from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

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

    sdf_file_name = 'TrueRobot.sdf'
    sdf = os.path.join(
        get_package_share_directory(package_description),
        'TrueRobot',
        sdf_file_name)
    
    print("Found UDRF")
    
    with open(sdf, 'r') as infp:
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

    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'TrueRobot.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])
    
    gui = ExecuteProcess(
            cmd=['ros2', 'run', 'joint_state_publisher_gui', 'joint_state_publisher_gui'],
            output='screen'
        )


    # create and return launch description object
    return LaunchDescription(
        [          
            rviz_node,  
            robot_state_publisher_node,
            gui
        ]
    )
