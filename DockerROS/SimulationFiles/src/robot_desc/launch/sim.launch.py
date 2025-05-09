import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node


def generate_launch_description():

    # # Specify the name of the package and path to xacro file within the package
    # pkg_name = 'robot_desc'
    # file_subpath = 'description/example_robot.urdf.xacro'

    # # Use xacro to process the file
    # xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    # robot_description_raw = xacro.process_file(xacro_file).toxml()

    package_description = "robot_desc"

    pkg_share_path = os.pathsep + os.path.join(get_package_prefix(package_description), 'share')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  pkg_share_path

    print("Fetching URDF ==>")

    sdf_file_name = 'simRobot.sdf'   
    sdf = os.path.join(
        get_package_share_directory(package_description),
        'robot',
        sdf_file_name)
    # sdf_file_name = 'robot.urdf'   
    # sdf = os.path.join(
    #     get_package_share_directory(package_description),
    #     'robot',
    #     sdf_file_name)
    
    world_file_name = 'simset.sdf'
    world_sdf = os.path.join(
        get_package_share_directory(package_description),
        'simWorld',
        world_file_name)

    with open(sdf, 'r') as infp:
        robot_desc = infp.read()
    
    print("Found UDRF")

    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'robot.rviz')


    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
        # parameters=[{'use_sim_time': True, 'robot_description': launch_ros.parameter_descriptions.ParameterValue( launch_ros.substitutions.Command(['xacro ', os.path.join(robot_description_package, 'robot/robot.udrf')]), value_type=str) }],
        output="screen"
    )

    gazebo = ExecuteProcess(
            cmd=['ign', 'gazebo', world_sdf],
            output='screen'
        )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                    arguments=['-name', 'robot',
                               '-topic', 'robot_description',
                               #'-world', world_sdf
                               ],
                    output='screen')

    # Run the node
    return LaunchDescription([
        rviz_node,
        gazebo,
        robot_state_publisher,
        # spawn_entity
    ])


