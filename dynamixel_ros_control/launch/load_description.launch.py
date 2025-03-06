from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    # Parameters
    port_name_arg = DeclareLaunchArgument(
        name='port_name',
        default_value='/dev/ttyUSB0',
        description='Path to USB serial converter device'
    )
    ld.add_action(port_name_arg)
    port_name = LaunchConfiguration('port_name')

    baud_rate_arg = DeclareLaunchArgument(
        name='baud_rate',
        default_value='57600',
        description='Baud rate'
    )
    ld.add_action(baud_rate_arg)
    baud_rate = LaunchConfiguration('baud_rate')

    id_arg = DeclareLaunchArgument(
        name='id',
        default_value='1',
        description='ID of the dynamixel to control'
    )
    ld.add_action(id_arg)
    id_arg = LaunchConfiguration('id')

    # Get the package directory
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('dynamixel_ros_control'), 'config', 'test_robot.urdf.xacro'
        ])

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file, ' port_name:=', port_name, ' baud_rate:=', baud_rate, ' id:=', id_arg,
                 ]),
                 value_type=str
    )

    # robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
        }],
    )
    ld.add_action(robot_state_publisher)

    return ld
