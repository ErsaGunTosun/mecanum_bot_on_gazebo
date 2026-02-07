import os 

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch import LaunchDescription


def generate_launch_description():
    spawn_pos = {'x': '0.0', 'y': '-3.5', 'z': '0.05', 'yaw': '1.57'}

    package_share_dir = get_package_share_directory("robot_simulation")

    world_path = PathJoinSubstitution([
        package_share_dir,
        'worlds',
        "easy.world"  
    ])
    

    ros_gz_sim = get_package_share_directory("gazebo_ros")

    launch_path = os.path.join(
            get_package_share_directory("robot_simulation"),
            "launch"
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items()
    )

    gz_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gzclient.launch.py')
        ),
        launch_arguments={
            'verbose': 'true'
        }.items()
    )
    

    robot_state_publisher = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_path,"robot_state_publisher.launch.py")
             )
    )

    spawn_entity = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_path,"spawn_entity.launch.py")
            ),
            launch_arguments={
                'x': spawn_pos['x'],
                'y': spawn_pos['y'],
                'z': spawn_pos['z'],
                'yaw': spawn_pos['yaw']
            }.items()
    )

    controllers = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_path,"controllers.launch.py")
            )
    )


    return LaunchDescription([
        gz_server,
        gz_client,
        robot_state_publisher,
        spawn_entity,
        controllers
    ])

