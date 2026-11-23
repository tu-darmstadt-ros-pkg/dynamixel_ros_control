"""The real dynamixel_ros_control plugin on a gz hosted bus, headless and without rendering."""

import os
import shutil
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import launch_testing
import launch_testing.actions
import pytest
import rclpy
from controller_manager_msgs.srv import (
    ListControllers,
    ListHardwareComponents,
    SetHardwareComponentState,
    SwitchController,
)
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix
from lifecycle_msgs.msg import State
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

CONFIG = Path(__file__).parent / "config"
# gz holds 3 kg at 0.5 m with a PH54-200, 4.81 Nm/A at the output (servo_models.yaml)
HOLDING_CURRENT = 3.0 * 9.81 * 0.5 / 4.81


@pytest.mark.launch_test
def generate_test_description():
    # Short, the SDK holds at most 99 characters of a port name
    directory = tempfile.mkdtemp(prefix="dxl_gz_", dir="/tmp")
    port = os.path.join(directory, "ports", "bus")
    robot = (CONFIG / "bus_rig.urdf").read_text().replace("@PORT@", port)
    robot_file = os.path.join(directory, "bus_rig.urdf")
    Path(robot_file).write_text(robot)
    world_file = os.path.join(directory, "bus_rig.sdf")
    Path(world_file).write_text(
        (CONFIG / "bus_rig.sdf").read_text().replace("@ROBOT@", robot_file)
    )
    gz_env = {"GZ_PARTITION": os.path.basename(directory), "GZ_IP": "127.0.0.1"}

    wait_for_port = ExecuteProcess(
        cmd=[
            PathJoinSubstitution(
                [
                    FindPackagePrefix("dynamixel_ros_control_gz"),
                    "lib",
                    "dynamixel_ros_control_gz",
                    "wait_for_ports",
                ]
            ),
            port,
        ],
        output="screen",
    )
    params = ["--param-file", str(CONFIG / "bus_rig_controllers.yaml")]
    controller_manager = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[str(CONFIG / "bus_rig_controllers.yaml")],
            # AdjustableOffsetTransmission keeps its offsets under HOME, which would outlive the test
            additional_env={"HOME": directory},
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "pendulum_controller",
                "gripper_controller",
                *params,
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["pendulum_torque_off_controller", "--inactive", *params],
        ),
    ]
    return (
        LaunchDescription(
            [
                ExecuteProcess(
                    cmd=["gz", "sim", "-s", "-r", world_file],
                    additional_env=gz_env,
                    output="screen",
                ),
                wait_for_port,
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=wait_for_port,
                        on_exit=lambda event, _: controller_manager
                        if event.returncode == 0
                        else Shutdown(reason="no port"),
                    )
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"gz_env": gz_env, "directory": directory},
    )


class TestGzBus(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("test_gz_bus")
        cls.positions = {}
        cls.efforts = {}

        def on_joint_states(message):
            cls.positions.update(zip(message.name, message.position))
            cls.efforts.update(zip(message.name, message.effort))

        cls.node.create_subscription(JointState, "joint_states", on_joint_states, 10)
        cls.pendulum = cls.node.create_publisher(
            Float64MultiArray, "pendulum_controller/commands", 10
        )
        cls.gripper = cls.node.create_publisher(
            Float64MultiArray, "gripper_controller/commands", 10
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_until(self, condition, timeout, message):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if condition():
                return
        self.fail(f"{message} (positions {self.positions}, efforts {self.efforts})")

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def call(self, service_type, name, request):
        client = self.node.create_client(service_type, name)
        self.assertTrue(client.wait_for_service(timeout_sec=30.0), name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=30.0)
        self.node.destroy_client(client)
        self.assertIsNotNone(future.result(), name)
        return future.result()

    def switch(self, activate, deactivate, strictness=SwitchController.Request.STRICT):
        request = SwitchController.Request(
            activate_controllers=activate,
            deactivate_controllers=deactivate,
            strictness=strictness,
        )
        self.assertTrue(
            self.call(
                SwitchController, "controller_manager/switch_controller", request
            ).ok,
            f"activate {activate}, deactivate {deactivate}",
        )

    def hardware_state(self):
        response = self.call(
            ListHardwareComponents,
            "controller_manager/list_hardware_components",
            ListHardwareComponents.Request(),
        )
        return response.component[0].state.id

    def configured(self, controller):
        response = self.call(
            ListControllers,
            "controller_manager/list_controllers",
            ListControllers.Request(),
        )
        return any(
            state.name == controller and state.state == "inactive"
            for state in response.controller
        )

    def command_pendulum(self, position, current):
        self.pendulum.publish(Float64MultiArray(data=[position, current]))

    def pendulum_holds(self, position, tolerance, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.assertAlmostEqual(
                self.positions["pendulum_joint"], position, delta=tolerance
            )

    def gz_power_cut(self, gz_env, cut):
        subprocess.run(
            [
                "gz",
                "topic",
                "-t",
                "/bus_rig/dynamixel/power_cut",
                "-m",
                "gz.msgs.Boolean",
                "-p",
                f"data: {'true' if cut else 'false'}",
            ],
            env={**os.environ, **gz_env},
            check=True,
        )

    def test_servos_on_physics(self, gz_env):
        self.wait_until(
            lambda: "gripper_joint" in self.positions
            and self.pendulum.get_subscription_count() > 0
            and self.gripper.get_subscription_count() > 0,
            90.0,
            "the controllers come up",
        )
        self.wait_until(
            lambda: self.configured("pendulum_torque_off_controller"),
            30.0,
            "torque off is loaded",
        )

        # Holding against gravity takes the physical current, homing and offset cancel
        self.command_pendulum(0.0, 5.0)
        self.wait_until(
            lambda: abs(self.positions["pendulum_joint"]) < 0.01, 30.0, "it holds at 0"
        )
        self.spin_for(1.0)
        self.pendulum_holds(0.0, 0.01, 1.0)
        self.assertAlmostEqual(
            self.efforts["pendulum_joint"],
            -HOLDING_CURRENT,
            delta=0.2 * HOLDING_CURRENT,
        )

        # Torque off is limp, it sags until the gearbox friction holds it
        self.switch(["pendulum_torque_off_controller"], ["pendulum_controller"])
        self.wait_until(
            lambda: self.positions["pendulum_joint"] > 0.3, 30.0, "limp, it sags"
        )
        self.spin_for(2.0)
        sagged = self.positions["pendulum_joint"]

        # Torque back holds where it sagged to
        self.switch(["pendulum_controller"], ["pendulum_torque_off_controller"])
        self.pendulum_holds(sagged, 0.02, 1.5)

        # Below the holding current the limit bites, and the reported current stays within it
        self.command_pendulum(0.0, 5.0)
        self.wait_until(
            lambda: abs(self.positions["pendulum_joint"]) < 0.01, 30.0, "back at 0"
        )
        self.command_pendulum(0.0, 0.3)
        self.wait_until(
            lambda: self.positions["pendulum_joint"] > 0.3,
            30.0,
            "it sags at 0.3 A",
        )
        self.spin_for(1.0)
        self.assertLessEqual(abs(self.efforts["pendulum_joint"]), 0.3 + 0.002)
        self.command_pendulum(0.0, 5.0)
        self.wait_until(
            lambda: abs(self.positions["pendulum_joint"]) < 0.01, 30.0, "back at 0"
        )

        # The fingers stall on the box and the servo draws its goal current
        self.gripper.publish(Float64MultiArray(data=[0.05, 0.5]))
        self.wait_until(
            lambda: abs(self.positions["gripper_joint"] - 0.02) < 0.0025,
            30.0,
            "the fingers reach the box",
        )
        self.spin_for(1.0)
        self.assertAlmostEqual(self.positions["gripper_joint"], 0.02, delta=0.0025)
        self.assertAlmostEqual(self.efforts["gripper_joint"], 0.5, delta=0.05)

        # A power cut faults the hardware and the pendulum falls, power and a request recover it
        self.gz_power_cut(gz_env, True)
        self.wait_until(
            lambda: self.hardware_state() != State.PRIMARY_STATE_ACTIVE,
            30.0,
            "a power cut faults the hardware",
        )
        self.spin_for(2.0)
        self.gz_power_cut(gz_env, False)
        self.spin_for(0.5)
        request = SetHardwareComponentState.Request(name="bus_rig")
        request.target_state.id = State.PRIMARY_STATE_ACTIVE
        self.assertTrue(
            self.call(
                SetHardwareComponentState,
                "controller_manager/set_hardware_component_state",
                request,
            ).ok
        )
        # Whichever of them the fault stopped
        self.switch(
            ["joint_state_broadcaster", "pendulum_controller"],
            [],
            SwitchController.Request.BEST_EFFORT,
        )
        self.positions.clear()
        self.wait_until(
            lambda: self.positions.get("pendulum_joint", 0.0) > 0.3,
            30.0,
            "it fell while unpowered",
        )
        self.pendulum_holds(self.positions["pendulum_joint"], 0.02, 1.5)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_port_is_removed(self, directory):
        self.addCleanup(shutil.rmtree, directory)
        self.assertFalse(os.path.exists(os.path.join(directory, "ports", "bus")))
