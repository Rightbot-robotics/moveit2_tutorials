from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("sherlock")
        .robot_description(file_path="config/truck_unloading_description.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="move_group_interface_tutorial",
        package="moveit2_tutorials",
        executable="move_group_interface_test",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ],
    )

    return LaunchDescription([move_group_demo])
