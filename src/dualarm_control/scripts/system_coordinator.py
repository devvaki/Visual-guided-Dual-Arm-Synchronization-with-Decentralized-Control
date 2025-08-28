#!/usr/bin/env python3
"""
System Coordinator - Orchestrates the entire dual-arm system (decentralized-safe)

PUB:
  /system_status    (std_msgs/String)  -> "INITIALIZING", "READY", "EXECUTING", "COMPLETED", "ERROR[:detail]"
  /emergency_stop   (std_msgs/Bool)    -> True when coordinator requests an e-stop

SUB:
  /task_status      (std_msgs/String)  -> Milestones emitted by subsystems:
        "ROBOT_A_READY"
        "ROBOT_B_READY"
        "VISION_READY"
        "MOTION_PATTERN_STARTED"
        "MOTION_PATTERN_COMPLETED"
        "ERROR" or "ERROR:<detail>"

PARAMS (ros2 param):
  ~readiness_timeout_sec        (float, default 30.0)   # to become READY
  ~overall_task_timeout_sec     (float, default 120.0)  # EXECUTING phase watchdog
  ~heartbeat_hz                 (float, default 1.0)    # publish /system_status at this rate
"""

import time
from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class SystemState(Enum):
    INITIALIZING = 0
    READY = 1
    EXECUTING = 2
    COMPLETED = 3
    ERROR = 4


class SystemCoordinator(Node):
    def __init__(self):
        super().__init__('system_coordinator')

        # ---- parameters ----
        self.readiness_timeout = float(self.declare_parameter('readiness_timeout_sec', 30.0).value)
        self.task_timeout = float(self.declare_parameter('overall_task_timeout_sec', 120.0).value)
        heartbeat_hz = float(self.declare_parameter('heartbeat_hz', 1.0).value)
        self.heartbeat_dt = 1.0 / max(0.1, heartbeat_hz)

        # ---- pubs/subs ----
        self.system_status_pub = self.create_publisher(String, '/system_status', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.task_status_sub = self.create_subscription(String, '/task_status', self.task_status_callback, 20)

        # ---- state ----
        self.state = SystemState.INITIALIZING
        self.state_since = time.time()

        self.robot_a_ready = False
        self.robot_b_ready = False
        self.vision_ready  = False

        # track when EXECUTING started
        self.exec_since = None
        self.last_error_detail = ""

        # heartbeat / watchdog
        self.timer = self.create_timer(self.heartbeat_dt, self._tick)

        self.get_logger().info("System Coordinator initialized")

    # ------------------- helpers -------------------
    def _pub_status(self, text: str):
        msg = String(); msg.data = text
        self.system_status_pub.publish(msg)

    def _set_state(self, new_state: SystemState, detail: str = ""):
        self.state = new_state
        self.state_since = time.time()
        if detail:
            self._pub_status(f"{new_state.name}:{detail}")
            self.get_logger().info(f"STATE -> {new_state.name} ({detail})")
        else:
            self._pub_status(new_state.name)
            self.get_logger().info(f"STATE -> {new_state.name}")

    def _estop(self, on: bool, reason: str = ""):
        msg = Bool(); msg.data = on
        self.emergency_stop_pub.publish(msg)
        if on:
            self.get_logger().error(f"E-STOP asserted. Reason: {reason}")
        else:
            self.get_logger().warn("E-STOP released")

    # ------------------- callbacks -------------------
    def task_status_callback(self, msg: String):
        data = msg.data.strip()
        # readiness flags
        if data == "ROBOT_A_READY":
            self.robot_a_ready = True
            self.get_logger().info("Robot A ready")
        elif data == "ROBOT_B_READY":
            self.robot_b_ready = True
            self.get_logger().info("Robot B ready")
        elif data == "VISION_READY":
            self.vision_ready = True
            self.get_logger().info("Vision ready")

        # motion milestones
        elif data == "MOTION_PATTERN_STARTED":
            if self.state == SystemState.READY:
                self.exec_since = time.time()
                self._set_state(SystemState.EXECUTING)
            else:
                self.get_logger().warn("Received MOTION_PATTERN_STARTED while not in READY")

        elif data == "MOTION_PATTERN_COMPLETED":
            self._set_state(SystemState.COMPLETED)

        # error
        elif data.startswith("ERROR"):
            self.last_error_detail = data[6:] if data.startswith("ERROR:") else ""
            self._set_state(SystemState.ERROR, self.last_error_detail)
            self._estop(True, self.last_error_detail or "unknown error")

        else:
            # Non-fatal info can be forwarded on /system_status if you like:
            self.get_logger().info(f"Task status: {data}")

    # ------------------- heartbeat / watchdog -------------------
    def _tick(self):
        now = time.time()

        # INITIALIZING -> READY when all subsystems are ready
        if self.state == SystemState.INITIALIZING:
            if self.robot_a_ready and self.robot_b_ready and self.vision_ready:
                self._set_state(SystemState.READY)
            elif (now - self.state_since) > self.readiness_timeout:
                self._set_state(SystemState.ERROR, "readiness timeout")
                self._estop(True, "readiness timeout")

        # EXECUTING watchdog
        elif self.state == SystemState.EXECUTING and self.exec_since is not None:
            if (now - self.exec_since) > self.task_timeout:
                self._set_state(SystemState.ERROR, "overall task timeout")
                self._estop(True, "overall task timeout")

        # Periodic heartbeat even if nothing changed
        self._pub_status(self.state.name if not self.last_error_detail else f"{self.state.name}:{self.last_error_detail}")


def main(args=None):
    rclpy.init(args=args)
    node = SystemCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
