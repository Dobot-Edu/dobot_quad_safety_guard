#!/usr/bin/env python3

import argparse
import math
import os
import threading
import time
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Float32, Header, String
from visualization_msgs.msg import Marker, MarkerArray

from .config import ensure_valid_cyclonedds_uri, load_yaml, resolve_path

@dataclass
class CameraIntrinsics:
    """用于将深度像素投影为点云的针孔相机内参。
    Pinhole camera intrinsics used to project depth pixels into point clouds.
    """

    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class DepthFrame:
    """用于点云定时发布的最近一帧深度数据。
    Latest depth frame used for scheduled point cloud publishing.
    """

    depth: np.ndarray
    scale: float
    frame_id: str
    stamp: object


class DepthBridgeNode(Node):
    """将 Dobot DDS 深度图桥接为 ROS 2 话题和障碍物距离。
    Bridge Dobot DDS depth images to ROS 2 topics and obstacle distances.
    """

    def __init__(self, config_file: str):
        super().__init__("dobot_depth_bridge_node")
        self._config_file = config_file
        self._config = load_yaml(config_file)

        dds_cfg = self._config.get("dds", {})
        safety_cfg = self._config.get("safety", {})
        viz_cfg = self._config.get("visualization", {})
        log_cfg = self._config.get("logging", {})

        self._front_topic = dds_cfg.get("front_depth_topic", "rt/camera/camera2/image_depth")
        self._back_topic = dds_cfg.get("back_depth_topic", "rt/camera/camera3/image_depth")
        self._dds_config = resolve_path(dds_cfg.get("config_file"), config_file)
        self._dds_domain_id = int(dds_cfg.get("domain_id", 0))
        self._cyclonedds_uri = ensure_valid_cyclonedds_uri(config_file)

        self._roi = safety_cfg.get("roi", {})
        self._front_danger = float(safety_cfg.get("front_danger_distance_m", 0.5))
        self._back_danger = float(safety_cfg.get("back_danger_distance_m", 0.5))
        self._min_depth = float(safety_cfg.get("min_valid_depth_m", 0.1))
        self._max_depth = float(safety_cfg.get("max_valid_depth_m", 3.0))
        self._method = str(safety_cfg.get("distance_method", "percentile")).lower()
        self._percentile = float(safety_cfg.get("distance_percentile", 10.0))
        self._distance_sample_step = max(1, int(safety_cfg.get("distance_sample_step", 2)))
        self._distance_publish_period = max(0.0, float(safety_cfg.get("distance_publish_period_seconds", 0.05)))
        self._log_period = float(log_cfg.get("distance_log_period_seconds", 1.0))
        self._last_log_time = 0.0

        intr_cfg = viz_cfg.get("camera_intrinsics", {})
        self._intrinsics = CameraIntrinsics(
            width=int(intr_cfg.get("width", 640)),
            height=int(intr_cfg.get("height", 480)),
            fx=float(intr_cfg.get("fx", 386.0)),
            fy=float(intr_cfg.get("fy", 386.0)),
            cx=float(intr_cfg.get("cx", 320.0)),
            cy=float(intr_cfg.get("cy", 240.0)),
        )
        pc_cfg = viz_cfg.get("pointcloud", {})
        img_cfg = viz_cfg.get("depth_image", {})
        marker_cfg = viz_cfg.get("markers", {})
        self._sample_step = max(1, int(pc_cfg.get("sample_step", 8)))
        self._max_points = max(100, int(pc_cfg.get("max_points", 12000)))
        self._pc_publish_period = max(0.0, float(pc_cfg.get("publish_period_seconds", 1.0)))
        self._pointcloud_roi_only = bool(pc_cfg.get("use_roi_only", True))
        self._image_publish_period = max(0.0, float(img_cfg.get("publish_period_seconds", 0.3)))
        self._marker_publish_period = max(0.0, float(marker_cfg.get("publish_period_seconds", 0.2)))
        self._publish_depth = bool(viz_cfg.get("publish_depth_image", True))
        self._publish_pc = bool(viz_cfg.get("publish_point_cloud", True))
        self._publish_markers = bool(viz_cfg.get("publish_markers", True))
        self._pointcloud_in_base = bool(viz_cfg.get("pointcloud_in_base_frame", True))
        self._front_frame = str(viz_cfg.get("front_frame_id", "front_depth_camera"))
        self._back_frame = str(viz_cfg.get("back_frame_id", "back_depth_camera"))
        self._marker_frame = str(viz_cfg.get("marker_frame_id", "safety_guard_base"))
        self._last_distance_process = {"front": 0.0, "back": 0.0}
        self._last_image_publish = {"front": 0.0, "back": 0.0}
        self._last_marker_publish = {"front": 0.0, "back": 0.0}
        self._last_pc_cache = {"front": 0.0, "back": 0.0}
        self._latest_pc_frames: dict[str, DepthFrame | None] = {"front": None, "back": None}
        self._pc_lock = threading.Lock()
        self._pc_event = threading.Event()
        self._pc_thread_stop = threading.Event()
        self._pc_thread = None
        self._pc_dirty = False
        self._sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._front_distance_pub = self.create_publisher(Float32, "/safety_guard/front/distance", self._sensor_qos)
        self._back_distance_pub = self.create_publisher(Float32, "/safety_guard/back/distance", self._sensor_qos)
        self._status_pub = self.create_publisher(String, "/safety_guard/depth_status", 10)
        self._markers_pub = self.create_publisher(MarkerArray, "/safety_guard/markers", self._sensor_qos)
        self._front_image_pub = self.create_publisher(Image, "/safety_guard/front/depth/image_raw", self._sensor_qos)
        self._back_image_pub = self.create_publisher(Image, "/safety_guard/back/depth/image_raw", self._sensor_qos)
        self._points_pub = self.create_publisher(PointCloud2, "/safety_guard/points", self._sensor_qos)
        if self._publish_pc:
            self._pc_thread = threading.Thread(target=self._point_cloud_worker, daemon=True)
            self._pc_thread.start()

        self._middleware = None
        self._init_dds()

        self.get_logger().info("前向深度 DDS 话题: %s" % self._front_topic)
        self.get_logger().info("后向深度 DDS 话题: %s" % self._back_topic)
        self.get_logger().info("CYCLONEDDS_URI: %s" % (self._cyclonedds_uri or "<unset>"))
        self.get_logger().info("DDS 配置: %s" % (self._dds_config or f"domain_id={self._dds_domain_id}"))

    def _init_dds(self):
        """创建前后深度图 DDS 订阅。
        Create DDS subscriptions for front and rear depth images.
        """
        import dds_middleware_python as dds

        if self._dds_config:
            self._middleware = dds.PyDDSMiddleware(self._dds_config)
        else:
            self._middleware = dds.PyDDSMiddleware(self._dds_domain_id)

        qos_config = {
            # 深度图是连续数据流，实时新帧比重传旧帧更重要。
            # Depth images are continuous data streams; real-time new frames are more important than retransmitting old frames.
            "reliability": "best_effort",
            "history_kind": "keep_last",
            "history_depth": 1,
            "durability": "volatile",
        }
        self._middleware.subscribeImage(
            self._front_topic,
            lambda msg: self._handle_depth("front", msg),
            qos_config,
        )
        self._middleware.subscribeImage(
            self._back_topic,
            lambda msg: self._handle_depth("back", msg),
            qos_config,
        )
        self.get_logger().info("点云发布: %s" % self._publish_pc)
        self.get_logger().info(
            "ROI: x=[%.2f, %.2f], y=[%.2f, %.2f], method=%s, percentile=%.1f"
            % (
                float(self._roi.get("x_min", 0.15)),
                float(self._roi.get("x_max", 0.85)),
                float(self._roi.get("y_min", 0.15)),
                float(self._roi.get("y_max", 0.85)),
                self._method,
                self._percentile,
            )
        )

    def _handle_depth(self, side: str, depth_msg):
        """处理单侧 DDS 深度帧并发布对应 ROS 2 输出。
        Process a single-side DDS depth frame and publish the corresponding ROS 2 outputs.
        """
        try:
            now = time.time()
            if not self._should_publish(self._last_distance_process, side, now, self._distance_publish_period):
                return

            depth, scale = self._depth_to_native(depth_msg)
            distance = self._compute_roi_distance(depth, scale)
            frame_id = self._front_frame if side == "front" else self._back_frame
            stamp = self.get_clock().now().to_msg()

            distance_msg = Float32()
            distance_msg.data = float(distance) if math.isfinite(distance) else float("nan")
            if side == "front":
                self._front_distance_pub.publish(distance_msg)
            else:
                self._back_distance_pub.publish(distance_msg)

            if self._publish_depth and self._should_publish(self._last_image_publish, side, now, self._image_publish_period):
                self._publish_depth_image(side, depth_msg, frame_id, stamp)
            if self._publish_pc and self._should_publish(self._last_pc_cache, side, now, self._pc_publish_period):
                self._cache_point_cloud_frame(side, depth, scale, frame_id, stamp)
            if self._publish_markers and self._should_publish(self._last_marker_publish, side, now, self._marker_publish_period):
                self._publish_marker(side, distance, stamp)

            if now - self._last_log_time >= self._log_period:
                self._last_log_time = now
                self.get_logger().info("%s depth distance: %.3fm" % (side, distance))

        except Exception as exc:
            status = String()
            status.data = f"{side} depth handling failed: {exc}"
            self._status_pub.publish(status)
            self.get_logger().error(status.data)

    def _should_publish(self, last_publish: dict[str, float], side: str, now: float, period: float) -> bool:
        if period <= 0.0 or now - last_publish.get(side, 0.0) >= period:
            last_publish[side] = now
            return True
        return False

    def _depth_to_native(self, depth_msg) -> tuple[np.ndarray, float]:
        """解析深度图原始数组，并返回换算到米的比例。
        Parse the raw depth image array and return the scale used to convert values to meters.
        """
        height = int(depth_msg.height())
        width = int(depth_msg.width())
        encoding = str(depth_msg.encoding()).upper()
        raw = np.asarray(depth_msg.data(), dtype=np.uint8)

        if "16UC1" in encoding or "MONO16" in encoding:
            return raw.view(np.uint16).reshape((height, width)), 0.001
        elif "32FC1" in encoding:
            return raw.view(np.float32).reshape((height, width)), 1.0
        else:
            raise ValueError(f"Unsupported depth encoding: {encoding}")

    def _compute_roi_distance(self, depth: np.ndarray, scale: float) -> float:
        """基于配置 ROI 内的有效深度样本计算障碍物距离。
        Calculate obstacle distance from valid depth samples inside the configured ROI.
        """
        h, w = depth.shape[:2]
        x0 = int(float(self._roi.get("x_min", 0.35)) * w)
        x1 = int(float(self._roi.get("x_max", 0.65)) * w)
        y0 = int(float(self._roi.get("y_min", 0.35)) * h)
        y1 = int(float(self._roi.get("y_max", 0.70)) * h)
        roi = depth[max(0, y0): min(h, y1): self._distance_sample_step, max(0, x0): min(w, x1): self._distance_sample_step]
        min_native = self._min_depth / scale
        max_native = self._max_depth / scale
        if np.issubdtype(roi.dtype, np.floating):
            valid = roi[np.isfinite(roi) & (roi >= min_native) & (roi <= max_native)]
        else:
            valid = roi[(roi >= min_native) & (roi <= max_native)]
        if valid.size == 0:
            return float("nan")
        if self._method == "min":
            return float(np.min(valid) * scale)
        percentile = min(100.0, max(0.0, self._percentile))
        kth = int((percentile / 100.0) * (valid.size - 1))
        return float(np.partition(valid, kth)[kth] * scale)

    def _publish_depth_image(self, side: str, depth_msg, frame_id: str, stamp):
        msg = Image()
        msg.header = Header(stamp=stamp, frame_id=frame_id)
        msg.height = int(depth_msg.height())
        msg.width = int(depth_msg.width())
        msg.encoding = str(depth_msg.encoding())
        msg.is_bigendian = int(depth_msg.is_bigendian())
        msg.step = int(depth_msg.step())
        msg.data = bytes(depth_msg.data())
        if side == "front":
            self._front_image_pub.publish(msg)
        else:
            self._back_image_pub.publish(msg)

    def _cache_point_cloud_frame(self, side: str, depth: np.ndarray, scale: float, frame_id: str, stamp):
        """只缓存最新原始深度帧，点云线程慢时直接覆盖旧帧。
        Cache only the latest raw depth frame, overwriting old frames when the point cloud thread is slow.
        """
        with self._pc_lock:
            self._latest_pc_frames[side] = DepthFrame(depth.copy(), scale, frame_id, stamp)
            self._pc_dirty = True
        self._pc_event.set()

    def _point_cloud_worker(self):
        """后台生成点云，确保 RViz 可视化不会阻塞 DDS 回调和避障距离发布。
        Generate point clouds in the background so RViz visualization does not block DDS callbacks or obstacle distance publishing.
        """
        while not self._pc_thread_stop.is_set():
            wait_time = self._pc_publish_period if self._pc_publish_period > 0.0 else 0.2
            self._pc_event.wait(wait_time)
            self._pc_event.clear()
            if self._pc_thread_stop.is_set():
                return
            self._publish_latest_point_cloud()

    def _publish_latest_point_cloud(self):
        """合并发布前后最新点云，减少 RViz 订阅和渲染负载。
        Merge and publish the latest front and rear point clouds to reduce RViz subscription and rendering load.
        """
        with self._pc_lock:
            if not self._pc_dirty:
                return
            frames = dict(self._latest_pc_frames)
            self._pc_dirty = False
        valid_frames = [(side, frame) for side, frame in frames.items() if frame is not None]
        if not valid_frames:
            return
        try:
            point_sets = [self._depth_to_points(frame.depth, frame.scale, side) for side, frame in valid_frames]
            points = np.concatenate(point_sets, axis=0) if len(point_sets) > 1 else point_sets[0]
            stamp = valid_frames[-1][1].stamp
            cloud = self._create_xyz32_cloud(Header(stamp=stamp, frame_id=self._marker_frame), points)
            self._points_pub.publish(cloud)
        except Exception as exc:
            status = String()
            status.data = f"point cloud publish failed: {exc}"
            self._status_pub.publish(status)
            self.get_logger().error(status.data)

    def _depth_to_points(self, depth: np.ndarray, scale: float, side: str) -> np.ndarray:
        """将采样深度像素投影到相机坐标系或演示基坐标系。
        Project sampled depth pixels into the camera coordinate frame or the demo base coordinate frame.
        """
        h, w = depth.shape[:2]
        if self._pointcloud_roi_only:
            x0 = max(0, int(float(self._roi.get("x_min", 0.35)) * w))
            x1 = min(w, int(float(self._roi.get("x_max", 0.65)) * w))
            y0 = max(0, int(float(self._roi.get("y_min", 0.35)) * h))
            y1 = min(h, int(float(self._roi.get("y_max", 0.70)) * h))
        else:
            x0, x1, y0, y1 = 0, w, 0, h

        depth_roi = depth[y0:y1, x0:x1]
        sampled_depth = depth_roi[0:depth_roi.shape[0]:self._sample_step, 0:depth_roi.shape[1]:self._sample_step]
        min_native = self._min_depth / scale
        max_native = self._max_depth / scale
        if np.issubdtype(sampled_depth.dtype, np.floating):
            valid_mask = np.isfinite(sampled_depth) & (sampled_depth >= min_native) & (sampled_depth <= max_native)
        else:
            valid_mask = (sampled_depth >= min_native) & (sampled_depth <= max_native)
        if not np.any(valid_mask):
            return np.empty((0, 3), dtype=np.float32)

        v_idx, u_idx = np.nonzero(valid_mask)
        if v_idx.size > self._max_points:
            v_idx = v_idx[: self._max_points]
            u_idx = u_idx[: self._max_points]

        z = sampled_depth[v_idx, u_idx].astype(np.float32, copy=False) * scale
        u = (x0 + u_idx * self._sample_step).astype(np.float32, copy=False)
        v = (y0 + v_idx * self._sample_step).astype(np.float32, copy=False)
        x_cam = (u - self._intrinsics.cx) * z / self._intrinsics.fx
        y_cam = (v - self._intrinsics.cy) * z / self._intrinsics.fy

        if self._pointcloud_in_base:
            # 演示基坐标系：+X 为前方，-X 为后方，+Y 为左侧，+Z 为上方。
            # Demo base coordinate frame: +X is front, -X is rear, +Y is left, and +Z is up.
            x = z if side == "front" else -z
            y = -x_cam
            point_z = -y_cam
        else:
            x = x_cam
            y = y_cam
            point_z = z

        return np.ascontiguousarray(np.column_stack((x, y, point_z)), dtype=np.float32)

    def _create_xyz32_cloud(self, header: Header, points: np.ndarray) -> PointCloud2:
        """直接使用 NumPy 内存构造点云，避免高密度点云产生大量 Python 对象。
        Build point clouds directly from NumPy memory to avoid creating many Python objects for high-density point clouds.
        """
        points = np.ascontiguousarray(points, dtype=np.float32)
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = int(points.shape[0])
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = False
        cloud.data = points.tobytes()
        return cloud

    def _publish_marker(self, side: str, distance: float, stamp):
        """发布 RViz 距离球和文字标签。
        Publish RViz distance spheres and text labels.
        """
        marker = Marker()
        marker.header = Header(stamp=stamp, frame_id=self._marker_frame)
        marker.ns = "safety_guard_distance"
        marker.id = 1 if side == "front" else 2
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        signed_distance = float(distance) if math.isfinite(distance) else 0.0
        marker.pose.position.x = signed_distance if side == "front" else -signed_distance
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.12
        marker.scale.y = 0.12
        marker.scale.z = 0.12
        marker.color.a = 0.9
        danger_distance = self._front_danger if side == "front" else self._back_danger
        in_danger = math.isfinite(distance) and distance <= danger_distance
        marker.color.r = 1.0 if in_danger else 0.0
        marker.color.g = 0.0 if in_danger else 1.0
        marker.color.b = 0.0

        text = Marker()
        text.header = Header(stamp=stamp, frame_id=self._marker_frame)
        text.ns = "safety_guard_label"
        text.id = 11 if side == "front" else 12
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = marker.pose.position.x
        text.pose.position.y = 0.0
        text.pose.position.z = 0.45
        text.pose.orientation.w = 1.0
        text.scale.z = 0.16
        text.color.a = 1.0
        text.color.r = marker.color.r
        text.color.g = marker.color.g
        text.color.b = marker.color.b
        label = "FRONT" if side == "front" else "BACK"
        dist_text = "nan" if not math.isfinite(distance) else f"{distance:.2f}m"
        text.text = f"{label} {dist_text}"

        arr = MarkerArray()
        arr.markers.append(marker)
        arr.markers.append(text)
        self._markers_pub.publish(arr)

    def destroy_node(self):
        self._pc_thread_stop.set()
        self._pc_event.set()
        if self._pc_thread is not None:
            self._pc_thread.join(timeout=1.0)
        with self._pc_lock:
            self._latest_pc_frames["front"] = None
            self._latest_pc_frames["back"] = None
        self._middleware = None
        super().destroy_node()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="safety_guard_config.yaml 配置文件路径 / Path to safety_guard_config.yaml")
    args = parser.parse_args(argv)

    rclpy.init()
    node = DepthBridgeNode(args.config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        # DDS 中间件内部可能持有原生线程；确保 Ctrl+C 后进程彻底退出，避免下次启动直接卡顿。
        # The DDS middleware may hold native threads; ensure the process exits completely after Ctrl+C to avoid freezing on the next startup.
        os._exit(0)


if __name__ == "__main__":
    main()
