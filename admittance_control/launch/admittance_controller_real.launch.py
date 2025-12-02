from pathlib import Path

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> launch.LaunchDescription:
    config_path = (
        Path(get_package_share_directory("admittance_control"))
        / "config"
        / "admittance_params_arm_only.yaml"
    )
    assert config_path.exists(), f"{__file__}: {config_path} does not exist."

    srdf_path = (
        Path(get_package_share_directory("neo_ur_moveit_config"))
        / "srdf"
        / "mpo_700.srdf.xacro"
    )
    assert srdf_path.exists(), f"{__file__}: {srdf_path} does not exist."

    moveit_config = (
        MoveItConfigsBuilder(robot_name="mpo_700", package_name="neo_ur_moveit_config")
        .robot_description_semantic(
            file_path=str(srdf_path),
            mappings={
                "prefix": "ur10e",
                "gripper_type": "vg10",
            },
        )
        .to_moveit_configs()
    )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package="admittance_control",
                executable="admittance_controller_node",
                output="screen",
                # prefix=["gdbserver localhost:3000"],
                parameters=[
                    str(config_path),
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ]
            )
        ]
    )
