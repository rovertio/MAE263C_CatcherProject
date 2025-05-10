import os
from ament_index_python.packages import get_package_share_directory
from ament_index_python import get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


from launch_ros.actions import Node
import xacro


def generate_launch_description():

    # # Specify the name of the package and path to xacro file within the package
    # pkg_name = 'robot_desc'
    # file_subpath = 'description/example_robot.urdf.xacro'

    # # Use xacro to process the file
    # xacro_file = os.path.join(get_package_share_directory(pkg_name),file_subpath)
    # robot_description_raw = xacro.process_file(xacro_file).toxml()

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

    sdf_file_name = 'robot.sdf'
    sdf = os.path.join(
        get_package_share_directory(package_description),
        'robot',
        sdf_file_name)

    with open(sdf, 'r') as infp:
        robot_desc = infp.read()
    
    print("Found UDRF")

    # # Configure the node
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'robot_description': robot_desc,
    #     'use_sim_time': True}] # add other parameters here if required
    # )

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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        )

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                    arguments=['-name', 'robot',
                        '-topic', 'robot_description'
                               ],
                    output='screen')

    # Run the node
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])


