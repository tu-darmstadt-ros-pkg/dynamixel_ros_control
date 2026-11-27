from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    ld = LaunchDescription()

    # Parameters
    port_name_arg = DeclareLaunchArgument(
        name="port_name",
        default_value="/dev/ttyACM0",
        # '/dev/ttyUSB0',
        description="Path to USB serial converter device",
    )
    ld.add_action(port_name_arg)
    port_name = LaunchConfiguration("port_name")

    baud_rate_arg = DeclareLaunchArgument(
        name="baud_rate", default_value="57600", description="Baud rate"
    )
    ld.add_action(baud_rate_arg)
    baud_rate = LaunchConfiguration("baud_rate")

    id_arg = DeclareLaunchArgument(
        name="id", default_value="16", description="ID of the dynamixel to control"
    )
    ld.add_action(id_arg)
    id_arg = LaunchConfiguration("id")

    id2_arg = DeclareLaunchArgument(
        name="id2",
        default_value="0",
        description=(
            "ID of a second dynamixel to put on the bus, or 0 for none. A second motor "
            "is what makes the multi-motor paths reachable: the sync read/write groups, "
            "the watchdog padding on a mixed bus, per-joint torque and partial securing. "
            "It is energised and read, but no controller commands it."
        ),
    )
    ld.add_action(id2_arg)
    id2 = LaunchConfiguration("id2")

    torque_on_startup_arg = DeclareLaunchArgument(
        name="torque_on_startup",
        default_value="true",
        description=(
            "Energise the motors on activation. Pass false to bring the bus up "
            "de-energised, which is how a first run on unfamiliar hardware should start."
        ),
    )
    ld.add_action(torque_on_startup_arg)
    torque_on_startup = LaunchConfiguration("torque_on_startup")

    velocity_limit_arg = DeclareLaunchArgument(
        name="velocity_limit",
        default_value="1.0",
        description="velocity_limit register written to both joints, in rad/s.",
    )
    ld.add_action(velocity_limit_arg)
    velocity_limit = LaunchConfiguration("velocity_limit")

    current_limit_arg = DeclareLaunchArgument(
        name="current_limit",
        default_value="0",
        description=(
            "current_limit register written to both joints, in A. 0 leaves the motor's "
            "own value alone. Set it for bench work: it is the firmware-side clamp "
            "behind the bench supply's."
        ),
    )
    ld.add_action(current_limit_arg)
    current_limit = LaunchConfiguration("current_limit")

    # Get the package directory
    urdf_file = PathJoinSubstitution(
        [
            get_package_share_directory("dynamixel_ros_control"),
            "config",
            "test_robot.urdf.xacro",
        ]
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                urdf_file,
                " port_name:=",
                port_name,
                " baud_rate:=",
                baud_rate,
                " id:=",
                id_arg,
                " id2:=",
                id2,
                " torque_on_startup:=",
                torque_on_startup,
                " velocity_limit:=",
                velocity_limit,
                " current_limit:=",
                current_limit,
            ]
        ),
        value_type=str,
    )

    # robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
            }
        ],
    )
    ld.add_action(robot_state_publisher)

    return ld
