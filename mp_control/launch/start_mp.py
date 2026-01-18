from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # Launch arguments
    # ============================================================
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # ============================================================
    # Paths
    # ============================================================
    urdf_path = PathJoinSubstitution([
        FindPackageShare("mp_description"),
        "urdf",
        "mp_description.urdf.xacro",
    ])

    controllers_path = PathJoinSubstitution([
        FindPackageShare("mp_control"),
        "config",
        "controllers.yaml",
    ])

    # ============================================================
    # Robot description (xacro → URDF)
    # ============================================================
    robot_description = {
        "robot_description": Command([
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_path,
        ])
    }

    # ============================================================
    # ros2_control node (hardware interface)
    # ============================================================
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            controllers_path,
            {"use_sim_time": use_sim_time},
        ],
        output="both",

        # Debug-friendly; remove prefix when done
        prefix=[
            "gdb ",
            "-ex break robot_interface_test::Robot7DoF::on_init ",
            "-ex break robot_interface_test::Robot7DoF::read ",
            "-ex break robot_interface_test::Robot7DoF::write ",
            "-ex run --args ",
        ],
    )

    # ============================================================
    # Robot State Publisher
    # ============================================================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time},
        ],
        output="both",
    )

    # ============================================================
    # Controller spawners
    # ============================================================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    mecanum_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Ensure proper startup order
    delay_mecanum_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[mecanum_controller_spawner],
        )
    )

    # ============================================================
    # Launch description
    # ============================================================
    return LaunchDescription([
        declare_use_sim_time,
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_mecanum_spawner,
    ])
