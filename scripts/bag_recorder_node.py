#!/usr/bin/env python3

import rospy
import rosbag
from threading import Lock
from std_msgs.msg import Bool
from datetime import datetime
from pathlib import Path as Pathlib

# echte Typen (WICHTIG für Stabilität)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

script_dir = Pathlib(__file__).parent


class MissionRecorder:

    def __init__(self):

        self.base_path = (script_dir / ".." / ".." / "Data" / "ros_bags").resolve()

        self.lock = Lock()
        self.recording = False
        self.bag = None

        # ---------------- TOPICS (typed, NO AnyMsg!) ----------------
        self.subscribers = []

        self.topic_types = {
            "/map_torsten_pose": PoseStamped,
            "/map_ur10_pose": PoseStamped,

            "/map_torsten_path": Path,
            "/map_ur10_path": Path,

            "/odom": Odometry,
            "/joint_states": JointState,

            "/ee_path_planned": Path,
            "/ee_path_actual": Path,

            "/tf": TFMessage,
            "/tf_static": TFMessage
        }

        for topic, msg_type in self.topic_types.items():
            if msg_type is None:
                continue

            cb = self._make_callback(topic)
            sub = rospy.Subscriber(topic, msg_type, cb, queue_size=10)
            self.subscribers.append(sub)

        # ---------------- CONTROL ----------------
        self.ctrl = rospy.Subscriber(
            "/recording_enable",
            Bool,
            self.control_callback,
            queue_size=1
        )

        rospy.loginfo("MissionRecorder ready")
        rospy.loginfo("Waiting for /recording_enable")

        rospy.sleep(0.5)  # wichtig für TF_static
        self.start()

    # ---------------- CALLBACK FACTORY ----------------

    def _make_callback(self, topic):

        def callback(msg):
            with self.lock:
                if not self.recording or self.bag is None:
                    return
                try:
                    self.bag.write(topic, msg, rospy.Time.now())
                except Exception as e:
                    rospy.logwarn(f"Bag write failed on {topic}: {e}")

        return callback

    # ---------------- CONTROL ----------------

    def control_callback(self, msg):
        if not msg.data:
            self.stop()

    def start(self):

        with self.lock:
            if self.recording:
                return

            rospy.loginfo(">>> START recording")

            ts = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            path = self.base_path / f"cube_recording_{ts}.bag"

            self.bag = rosbag.Bag(str(path), 'w', compression=rosbag.Compression.NONE)
            self.recording = True

    def stop(self):

        rospy.loginfo("<<< STOP recording")

        # 1. stop callbacks FIRST
        with self.lock:
            self.recording = False

        # 2. allow callback drain (IMPORTANT)
        rospy.sleep(0.3)

        # 3. close bag safely
        with self.lock:
            if self.bag is not None:
                try:
                    self.bag.close()
                except Exception as e:
                    rospy.logwarn(f"Error closing bag: {e}")

                self.bag = None

        # 4. shutdown node cleanly
        rospy.signal_shutdown("Recording finished")

    # ---------------- SHUTDOWN ----------------

    def shutdown(self):
        rospy.loginfo("Shutting down recorder...")
        self.stop()


if __name__ == "__main__":
    rospy.init_node("mission_recorder")

    node = MissionRecorder()
    rospy.on_shutdown(node.shutdown)

    rospy.spin()