from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    # Get the package directory
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('dynamixel_ros_control'), 'config', 'test_robot.urdf.xacro'
        ])

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file,
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
