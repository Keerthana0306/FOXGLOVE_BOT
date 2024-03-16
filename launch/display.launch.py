import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def find_root_directory():
    current_file_path = os.path.abspath(__file__)
    root_directory = os.path.dirname(current_file_path)

    while root_directory != os.path.dirname(root_directory):
        root_directory = os.path.dirname(root_directory)

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='sam_bot_description').find('sam_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/sam_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    rviz_config_file = os.path.join(
        get_package_share_directory('sam_bot_description'),  # Replace with your RViz package name
        'rviz',  # Assuming RViz configuration file is stored in a 'rviz' directory within your package
        'camera_visualization.rviz'  # Replace with your RViz configuration file name
    )
    world_path=os.path.join(pkg_share, 'world/my_world.sdf'),
    foxglove_bridge_launch_file = os.path.join(
        find_root_directory(),
        'path', 'to', 'foxglove_bridge', 'launch', 'foxglove_bridge.launch.py'
    )
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    camera_node = launch_ros.actions.Node(
        package='sam_bot_description',  # Replace with your camera package name
        executable='camera.py', 
        name='camera_publisher', # Assuming your camera node is named camera_publisher
        output='screen'
    )
    spawn_entity = launch_ros.actions.Node(
  package='gazebo_ros',
  executable='spawn_entity.py',
  arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
  output='screen'
)
    robot_localization_node = launch_ros.actions.Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       output='screen',
       parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': LaunchConfiguration('use_sim_time')}]
)

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(foxglove_bridge_launch_file),
            launch_arguments={
                'port': '8765',
                # Add other arguments here if needed
            }.items(),
        ),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        spawn_entity,
        robot_localization_node,
        rviz_node,
        camera_node
    ])