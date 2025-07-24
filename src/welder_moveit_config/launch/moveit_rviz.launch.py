from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("welder", package_name="welder_moveit_config")
        .robot_description(file_path="config/welder.urdf.xacro")
        .robot_description_semantic(file_path="config/welder.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines("ompl")
        .to_moveit_configs()
    )

    # 🔸 kinematics.yaml 경로 추가 (직접 로딩)
    kinematics_yaml_path = moveit_config.package_path / "config" / "kinematics.yaml"
    import yaml

    with open(kinematics_yaml_path, "r") as f:
        kinematics_yaml = yaml.safe_load(f)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.trajectory_execution,
            moveit_config.planning_pipelines,
            {"robot_description_kinematics": kinematics_yaml},  # 🔸 직접 주입
            {"use_sim_time": True},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", str(moveit_config.package_path / "launch/moveit.rviz")],
    )

    return LaunchDescription([move_group_node, rviz_node])
