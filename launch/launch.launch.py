from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
## add realsense camer alaunch if you are going to launch camera on your system
    pose_detection_cmd = Node(
        package="pose_detection",
        executable="pose_detection_node",
        output="both"
    )

    # particle_filter_cmd = Node(
    #     package="particle_filter",
    #     executable="particle_filter_node",
    #     output="both"
    # )

    ## only when run alone not with entire system
    smartthings_node_cmd = Node(
        package='smartthings_ros',
        executable='smartthings_node',
        output='screen'
    )
    ## only when run alone not with entire system
    aptags_world_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('aptags_tf_broadcast'), 'launch', 'tf_broadcast.launch.py']))
    )


    ## TODO: add apriltag detection that only reads one image
    aptag_det_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            get_package_share_directory('apriltag_ros'), 'launch', 'tag_realsense.launch.py']))
    )

    ld.add_action(pose_detection_cmd)
    ld.add_action(aptags_world_cmd)
    # ld.add_action(particle_filter_cmd)
    ld.add_action(smartthings_node_cmd)
    ld.add_action(aptag_det_cmd)

    return ld
