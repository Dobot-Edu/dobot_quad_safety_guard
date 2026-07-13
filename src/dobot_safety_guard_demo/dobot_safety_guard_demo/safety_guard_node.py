#!/usr/bin/env python3

import argparse
import math
import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String

from .config import load_yaml


@dataclass
class DistanceSample:
    """最近一次障碍物距离及接收时间。
    Most recent obstacle distance and receive timestamp.
    """

    value: float = float("nan")
    stamp: float = 0.0


class SafetyGuardNode(Node):
    """根据前后障碍物距离发布安全状态。
    Publish safety states based on front and rear obstacle distances.

    机器人运动由遥控器控制。本节点不发送运动指令，仅开启机器狗本体
    Robot motion is controlled by the remote controller. This node does not send motion commands;
    内置避障，并基于深度感知结果输出安全状态。
    it only enables the quadruped robot's built-in obstacle avoidance and outputs safety states based on depth perception results.
    """

    SAFE = "SAFE"
    FRONT_DANGER = "FRONT_DANGER"
    BACK_DANGER = "BACK_DANGER"
    BOTH_DANGER = "BOTH_DANGER"
    RECOVERING = "RECOVERING"
    SENSOR_STALE = "SENSOR_STALE"

    def __init__(self, config_file: str, grpc_addr: str | None):
        super().__init__("dobot_safety_guard_node")
        self._config = load_yaml(config_file)
        robot_cfg = self._config.get("robot", {})
        safety_cfg = self._config.get("safety", {})
        log_cfg = self._config.get("logging", {})

        self._grpc_addr = grpc_addr or robot_cfg.get("grpc_addr", "192.168.5.2:50051")
        self._enable_builtin_oa = bool(robot_cfg.get("enable_builtin_obstacle_avoidance", True))

        self._front_danger = float(safety_cfg.get("front_danger_distance_m", 0.5))
        self._front_recover = float(safety_cfg.get("front_recover_distance_m", 0.7))
        self._back_danger = float(safety_cfg.get("back_danger_distance_m", 0.5))
        self._back_recover = float(safety_cfg.get("back_recover_distance_m", 0.5))
        self._recover_stable_seconds = float(safety_cfg.get("recover_stable_seconds", 2.0))
        self._stale_timeout = float(safety_cfg.get("stale_timeout_seconds", 1.0))
        self._safety_log_period = float(log_cfg.get("safety_log_period_seconds", 0.5))
        self._danger_log_period = float(log_cfg.get("danger_log_period_seconds", 1.0))

        self._front = DistanceSample()
        self._back = DistanceSample()
        self._state = self.SENSOR_STALE
        self._safe_since: float | None = None
        self._robot = None
        self._last_safety_log_time = 0.0
        self._last_danger_log_time = 0.0
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._state_pub = self.create_publisher(String, "/safety_guard/state", 10)
        self._event_pub = self.create_publisher(String, "/safety_guard/events", 10)
        self.create_subscription(Float32, "/safety_guard/front/distance", self._front_cb, sensor_qos)
        self.create_subscription(Float32, "/safety_guard/back/distance", self._back_cb, sensor_qos)
        self.create_timer(0.1, self._tick)

        self._connect_robot()
        self.get_logger().info("安全状态机已启动，gRPC: %s" % self._grpc_addr)
        self.get_logger().info(
            "阈值: front<=%.2fm danger, front>%.2fm recover, back<=%.2fm danger, back>%.2fm recover"
            % (self._front_danger, self._front_recover, self._back_danger, self._back_recover)
        )
        self.get_logger().info("当前运动由遥控器控制，本节点仅负责感知、显示和安全状态提示。")

    def _connect_robot(self):
        """连接机器人，并按配置开启本体内置避障。
        Connect to the robot and enable built-in obstacle avoidance according to the configuration.
        """
        try:
            from dobot_quad import RobotClient

            self._robot = RobotClient(self._grpc_addr)
            if self._enable_builtin_oa:
                res = self._robot.set_obstacle_avoidance(True)
                self.get_logger().info(
                    "已开启机器狗本体内置避障: %s" % getattr(res, "current_enabled", True)
                )
        except Exception as exc:
            self._robot = None
            self.get_logger().error("连接机器人或开启内置避障失败: %s" % str(exc))
            self.get_logger().warning("节点仍会输出距离、状态和 Rviz，可在连接恢复后重启节点。")

    def _front_cb(self, msg: Float32):
        self._front = DistanceSample(float(msg.data), time.time())

    def _back_cb(self, msg: Float32):
        self._back = DistanceSample(float(msg.data), time.time())

    def _tick(self):
        """周期性评估安全状态并发布状态话题。
        Periodically evaluate the safety state and publish the state topic.
        """
        now = time.time()
        next_state = self._evaluate_state(now)
        if next_state != self._state:
            self._transition(next_state)

        self._log_realtime_distances(now)
        self._log_active_danger(now)

        state_msg = String()
        state_msg.data = self._state
        self._state_pub.publish(state_msg)

    def _evaluate_state(self, now: float) -> str:
        """评估安全状态机；危险距离优先于传感器超时判断。
        Evaluate the safety state machine; danger distance has priority over sensor timeout checks.
        """
        front_danger = self._is_distance_le(self._front.value, self._front_danger)
        back_danger = self._is_distance_le(self._back.value, self._back_danger)
        if front_danger and back_danger:
            self._safe_since = None
            return self.BOTH_DANGER
        if front_danger:
            self._safe_since = None
            return self.FRONT_DANGER
        if back_danger:
            self._safe_since = None
            return self.BACK_DANGER

        front_stale = self._is_stale(self._front, now)
        back_stale = self._is_stale(self._back, now)
        if front_stale or back_stale:
            self._safe_since = None
            return self.SENSOR_STALE

        recovered = self._is_distance_gt(self._front.value, self._front_recover) and self._is_distance_gt(
            self._back.value, self._back_recover
        )
        if not recovered:
            self._safe_since = None
            return self.RECOVERING

        if self._state in (self.FRONT_DANGER, self.BACK_DANGER, self.BOTH_DANGER, self.RECOVERING):
            # 恢复阶段需要稳定计时，避免距离在阈值附近抖动导致状态频繁切换。
            # The recovery phase requires stable timing to avoid frequent state switching when distances fluctuate near thresholds.
            if self._safe_since is None:
                self._safe_since = now
                return self.RECOVERING
            if now - self._safe_since < self._recover_stable_seconds:
                return self.RECOVERING

        return self.SAFE

    def _transition(self, next_state: str):
        old_state = self._state
        self._state = next_state
        event = (
            f"{old_state} -> {next_state}; front={self._front.value:.3f}m, "
            f"back={self._back.value:.3f}m"
        )
        self.get_logger().info(event)
        msg = String()
        msg.data = event
        self._event_pub.publish(msg)

        if next_state in (self.FRONT_DANGER, self.BACK_DANGER, self.BOTH_DANGER):
            self.get_logger().warning("检测到危险，请通过遥控器停止或远离障碍方向: %s" % next_state)

    def _log_realtime_distances(self, now: float):
        if self._safety_log_period <= 0.0:
            return
        if now - self._last_safety_log_time < self._safety_log_period:
            return
        self._last_safety_log_time = now
        self.get_logger().info(
            "实时距离: front=%.3fm, back=%.3fm, state=%s"
            % (self._front.value, self._back.value, self._state)
        )

    def _log_active_danger(self, now: float):
        if self._danger_log_period <= 0.0:
            return
        if self._state not in (self.FRONT_DANGER, self.BACK_DANGER, self.BOTH_DANGER):
            return
        if now - self._last_danger_log_time < self._danger_log_period:
            return
        self._last_danger_log_time = now
        self.get_logger().warning(
            "检测到危险，请通过遥控器停止或远离障碍方向: %s" % self._state
        )

    def _is_stale(self, sample: DistanceSample, now: float) -> bool:
        return sample.stamp <= 0.0 or now - sample.stamp > self._stale_timeout

    @staticmethod
    def _is_distance_le(value: float, threshold: float) -> bool:
        return math.isfinite(value) and value <= threshold

    @staticmethod
    def _is_distance_gt(value: float, threshold: float) -> bool:
        return math.isfinite(value) and value > threshold

    def destroy_node(self):
        if self._robot is not None:
            try:
                self._robot.close()
            except Exception:
                pass
        super().destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="safety_guard_config.yaml 配置文件路径 / Path to safety_guard_config.yaml")
    parser.add_argument("--grpc-addr", default=None, help="覆盖配置文件中的机器人 gRPC 地址 / Override the robot gRPC address in the configuration file")
    args = parser.parse_args(argv)

    rclpy.init()
    node = SafetyGuardNode(args.config, args.grpc_addr)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
