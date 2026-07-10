import socket
import time
import unittest

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from tf2_msgs.msg import TFMessage


def _unused_udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp_socket:
        udp_socket.bind(("127.0.0.1", 0))
        return udp_socket.getsockname()[1]


TEST_PORT = _unused_udp_port()
IDENTITY_ROTATION = "1 0 0 0 1 0 0 0 1"


@pytest.mark.launch_test
def generate_test_description():
    dtrack_node = launch_ros.actions.Node(
        package="dtrack_publisher",
        executable="dtrack_multi_body_node",
        name="dtrack_multi_body_node_test",
        output="screen",
        parameters=[
            {
                "udp_port": TEST_PORT,
                "frame_id": "test_mocap",
                "parent_frame": "test_odom",
                "max_path_length": 2,
            }
        ],
    )

    return (
        launch.LaunchDescription(
            [
                dtrack_node,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"dtrack_node": dtrack_node},
    )


class TestDTrackIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("dtrack_integration_test")
        self.poses = []
        self.paths = []
        self.transforms = []
        self.node.create_subscription(
            PoseStamped,
            "/dtrack/body_1/pose",
            self.poses.append,
            10,
        )
        self.node.create_subscription(
            Path,
            "/dtrack/body_1/path",
            self.paths.append,
            10,
        )
        self.node.create_subscription(
            TFMessage,
            "/tf",
            self.transforms.append,
            100,
        )
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def tearDown(self):
        self.udp_socket.close()
        self.node.destroy_node()

    def _packet(self, x_mm):
        return (
            f"6d 1 [0 0.8] [{x_mm} 2000 3000 0 0 0] "
            f"[{IDENTITY_ROTATION}]\n"
        ).encode()

    def _send(self, packet):
        self.udp_socket.sendto(packet, ("127.0.0.1", TEST_PORT))

    def _spin_until(self, predicate, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail("Timed out while waiting for ROS 2 data")

    def _send_until_position(self, x_mm):
        expected_x = x_mm / 1000.0
        deadline = time.monotonic() + 8.0
        next_send = 0.0
        while time.monotonic() < deadline:
            current_time = time.monotonic()
            if current_time >= next_send:
                self._send(self._packet(x_mm))
                next_send = current_time + 0.2
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.poses and abs(
                self.poses[-1].pose.position.x - expected_x
            ) < 1e-9:
                return
        self.fail("Timed out while waiting for a DTrack pose")

    def test_udp_pose_path_and_dynamic_transform(self):
        self._send_until_position(1000)
        self._send_until_position(2000)
        self._send_until_position(3000)
        self._spin_until(
            lambda: bool(self.paths)
            and bool(self.paths[-1].poses)
            and abs(self.paths[-1].poses[-1].pose.position.x - 3.0) < 1e-9
            and bool(self.transforms)
        )

        pose = self.poses[-1]
        self.assertEqual(pose.header.frame_id, "test_mocap")
        self.assertAlmostEqual(pose.pose.position.x, 3.0)
        self.assertAlmostEqual(pose.pose.position.y, 2.0)
        self.assertAlmostEqual(pose.pose.position.z, 3.0)
        self.assertAlmostEqual(pose.pose.orientation.x, 0.0)
        self.assertAlmostEqual(pose.pose.orientation.y, 0.0)
        self.assertAlmostEqual(pose.pose.orientation.z, 0.0)
        self.assertAlmostEqual(pose.pose.orientation.w, 1.0)

        path = self.paths[-1]
        self.assertEqual(path.header.frame_id, "test_mocap")
        self.assertEqual(len(path.poses), 2)
        self.assertAlmostEqual(path.poses[-1].pose.position.x, 3.0)

        matching_transforms = [
            transform
            for message in self.transforms
            for transform in message.transforms
            if transform.header.frame_id == "test_odom"
            and transform.child_frame_id == "test_mocap"
        ]
        self.assertTrue(matching_transforms)
        transform = matching_transforms[-1].transform
        self.assertAlmostEqual(transform.translation.x, 0.0)
        self.assertAlmostEqual(transform.translation.y, 0.0)
        self.assertAlmostEqual(transform.translation.z, 0.0)
        self.assertAlmostEqual(transform.rotation.w, 1.0)

    def test_invalid_body_does_not_create_publishers(self):
        invalid_packet = (
            "6d 1 [9 1] [1000 2000 3000 0 0 0] [1 0 0]\n"
        ).encode()
        self._send(invalid_packet)
        end_time = time.monotonic() + 1.5
        while time.monotonic() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertEqual(
            self.node.count_publishers("/dtrack/body_10/pose"),
            0,
        )


@launch_testing.post_shutdown_test()
class TestDTrackProcess(unittest.TestCase):
    def test_exit_code(self, proc_info, dtrack_node):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            process=dtrack_node,
        )
