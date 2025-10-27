#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ROS 2 (Humble) node that listens to Google Cloud Pub/Sub subscriptions and logs messages,
matching the behavior of the original script.

Parameters:
- project_id (string)
- gps_subscription (string)
- utm_subscription (string)
- colmap_subscription (string)
- key_file_paths (string array)
- mode (string) one of: gps, utm, colmap, pull-gps, pull-utm, pull-colmap
- pull_limit (int)
- publish_ros (bool) if true, republishes parsed JSON to std_msgs/String topics

Topics (published if publish_ros=True):
- /cviss/waypoints/gps        (std_msgs/msg/String, JSON)
- /cviss/waypoints/utm        (std_msgs/msg/String, JSON)
- /cviss/waypoints/colmap     (std_msgs/msg/String, JSON)
"""

import os
import json
import threading
from pathlib import Path
from typing import Optional, List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from google.cloud import pubsub_v1
from google.api_core.exceptions import GoogleAPICallError


def pretty_line(char="=", n=60):
    return char * n


class PubSubListener(Node):
    def __init__(self):
        super().__init__("cviss_pubsub_listener")

        # ---- Parameters ----
        self.declare_parameter("project_id", "cviss-waypoint-control")
        self.declare_parameter("gps_subscription", "gps-sub")
        self.declare_parameter("utm_subscription", "utm-sub")
        self.declare_parameter("colmap_subscription", "colmap-sub")
        self.declare_parameter("key_file_paths", [
            "../key/ros-team-key.json",
            "./ros-team-key.json",
            "../ros-team-key.json",
        ])
        self.declare_parameter("mode", "gps")
        self.declare_parameter("pull_limit", 5)
        self.declare_parameter("publish_ros", True)

        self.project_id: str = self.get_parameter("project_id").get_parameter_value().string_value
        self.gps_subscription: str = self.get_parameter("gps_subscription").get_parameter_value().string_value
        self.utm_subscription: str = self.get_parameter("utm_subscription").get_parameter_value().string_value
        self.colmap_subscription: str = self.get_parameter("colmap_subscription").get_parameter_value().string_value
        self.mode: str = self.get_parameter("mode").get_parameter_value().string_value.lower()
        self.pull_limit: int = self.get_parameter("pull_limit").get_parameter_value().integer_value
        self.publish_ros: bool = self.get_parameter("publish_ros").get_parameter_value().bool_value

        key_paths_param = self.get_parameter("key_file_paths").get_parameter_value().string_array_value
        self.key_file_paths: List[str] = list(key_paths_param) if key_paths_param else []

        # ---- Auth discovery ----
        key_file = self._find_key_file(self.key_file_paths)
        if key_file:
            os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = key_file
            self.get_logger().info(f"🔑 Key file found: {key_file}")
        else:
            self.get_logger().warn(
                "Key file not found in any of the expected paths. "
                "Will try using existing gcloud auth / default credentials."
            )

        # ---- Pub/Sub client ----
        self.subscriber = pubsub_v1.SubscriberClient()

        # ---- Optional ROS publishers ----
        self.pub_gps = self.create_publisher(Float64MultiArray, "/mavic_1/towereye_wp", 10) if self.publish_ros else None
        self.pub_utm = self.create_publisher(Float64MultiArray, "/cviss/waypoints/utm", 10) if self.publish_ros else None
        self.pub_colmap = self.create_publisher(Float64MultiArray, "/cviss/waypoints/colmap", 10) if self.publish_ros else None

        # ---- Control for streaming futures ----
        self._streaming_future = None
        self._streaming_lock = threading.Lock()

        # ---- Start mode ----
        self._start_mode(self.mode)

        # Clean shutdown hook
        self.add_on_set_parameters_callback(self._on_param_change)

    # ------------- Helpers -------------

    def _find_key_file(self, candidates: List[str]) -> Optional[str]:
        for p in candidates:
            pp = Path(p).expanduser().resolve()
            if pp.exists():
                return str(pp)
        return None

    def _subscription_path(self, sub_id: str) -> str:
        return self.subscriber.subscription_path(self.project_id, sub_id)

    def _cancel_streaming(self):
        with self._streaming_lock:
            if self._streaming_future is not None:
                self._streaming_future.cancel()
                self._streaming_future = None
                
        # ---------- Batch helpers ----------
    @staticmethod
    def _to_float(v, default=0.0):
        try:
            return float(v)
        except (TypeError, ValueError):
            return float(default)

    @staticmethod
    def _iter_payload_items(data):
        """
        Normalize incoming payloads to an iterable of 'items', each containing
        either a 'coordinates' dict or the fields directly.

        Supports:
        - Single object with 'coordinates': {...}
        - Single object with the fields directly
        - Array of such objects (top-level list)
        - Object with arrays under keys: 'items', 'batch', 'waypoints'
        - Object with 'coordinates' being a list of coordinate dicts
        """
        # 1) List at top level
        if isinstance(data, list):
            for itm in data:
                yield itm
            return

        # 2) Known list-like container keys
        for key in ("items", "batch", "waypoints", "messages"):
            if isinstance(data, dict) and isinstance(data.get(key), list):
                for itm in data[key]:
                    yield itm
                return

        # 3) coordinates is a list
        if isinstance(data, dict) and isinstance(data.get("coordinates"), list):
            for c in data["coordinates"]:
                # wrap each coordinate dict into an item shape
                yield {"coordinates": c}
            return

        # 4) Fallback: treat the whole object as a single item
        yield data


    # ------------- Mode management -------------

    def _start_mode(self, mode: str):
        if mode == "gps":
            self._subscribe_stream(self.gps_subscription, self._cb_gps, "🚁 GPS Message Streaming - For Drone")
        elif mode == "utm":
            self._subscribe_stream(self.utm_subscription, self._cb_utm, "🤖 UTM Message Streaming - For Robot")
        elif mode == "colmap":
            self._subscribe_stream(self.colmap_subscription, self._cb_colmap, "🤖 COLMAP Message Streaming - Original")
        elif mode == "pull-gps":
            self._pull_once(self.gps_subscription)
            rclpy.shutdown()
        elif mode == "pull-utm":
            self._pull_once(self.utm_subscription)
            rclpy.shutdown()
        elif mode == "pull-colmap":
            self._pull_once(self.colmap_subscription)
            rclpy.shutdown()
        else:
            self.get_logger().error(f"Unknown mode: {mode}")

    def _subscribe_stream(self, sub_id: str, cb, banner: str):
        path = self._subscription_path(sub_id)
        self.get_logger().info(pretty_line())
        self.get_logger().info(banner)
        self.get_logger().info(pretty_line())
        self.get_logger().info(f"Project:      {self.project_id}")
        self.get_logger().info(f"Subscription: {sub_id}")
        self.get_logger().info(f"Path:         {path}")
        self.get_logger().info("🎧 Listening for messages... (Ctrl+C to stop)")

        with self._streaming_lock:
            self._streaming_future = self.subscriber.subscribe(path, callback=cb)

    def _pull_once(self, sub_id: str):
        path = self._subscription_path(sub_id)
        self.get_logger().info(pretty_line())
        self.get_logger().info(f"📥 Pulling Messages - {sub_id}")
        self.get_logger().info(pretty_line())

        try:
            resp = self.subscriber.pull(
                request={"subscription": path, "max_messages": int(self.pull_limit)}
            )

            if not resp.received_messages:
                self.get_logger().info("📭 No messages available")
                self.get_logger().info(pretty_line())
                return

            self.get_logger().info(f"📬 Received {len(resp.received_messages)} message(s):")
            ack_ids = []
            for i, rm in enumerate(resp.received_messages, 1):
                try:
                    data = json.loads(rm.message.data.decode("utf-8"))
                except Exception as e:
                    self.get_logger().error(f"[{i}] Failed to decode JSON: {e}")
                    continue

                waypoint_id = data.get("waypointId", "N/A")
                coords = data.get("coordinates", {})
                self.get_logger().info(f"[{i}] {waypoint_id}")
                self.get_logger().info(json.dumps(coords, indent=4))
                ack_ids.append(rm.ack_id)

            if ack_ids:
                self.subscriber.acknowledge(request={"subscription": path, "ack_ids": ack_ids})
                self.get_logger().info("✅ All messages acknowledged")
        except GoogleAPICallError as e:
            self.get_logger().error(f"❌ API Error: {e}")
        except Exception as e:
            self.get_logger().error(f"❌ Error: {e}")
        finally:
            self.get_logger().info(pretty_line())

    # ------------- Callbacks -------------

    def _cb_gps(self, message: pubsub_v1.subscriber.message.Message):
        try:
            data = json.loads(message.data.decode("utf-8"))

            flat = []  # concatenated output
            count = 0
            for item in self._iter_payload_items(data):
                coords = item.get("coordinates", item) or {}

                lon = self._to_float(coords.get("longitude", 0.0))
                lat = self._to_float(coords.get("latitude", 0.0))
                alt = self._to_float(coords.get("altitude", 0.0))
                azm = self._to_float(coords.get("azimuth", 0.0))

                flat.extend([lon, lat, alt, azm])
                count += 1

            self.get_logger().info("\n" + pretty_line())
            self.get_logger().info(f"📡 [GPS Message Received] items={count}")
            self.get_logger().info(pretty_line())
            if count == 1:
                self.get_logger().info(f"Single: [lon,lat,alt,az]= {flat}")
            else:
                self.get_logger().info(f"Batch flattened length={len(flat)}")

            if self.publish_ros and self.pub_gps is not None:
                msg = Float64MultiArray()
                msg.data = flat
                self.pub_gps.publish(msg)
                self.get_logger().info(f"📤 Published /mavic_1/towereye_wp: {msg.data}")

            message.ack()

        except Exception as e:
            self.get_logger().error(f"❌ Error occurred in _cb_gps: {e}")
            message.nack()

    def _cb_utm(self, message: pubsub_v1.subscriber.message.Message):
        try:
            data = json.loads(message.data.decode("utf-8"))

            flat = []
            count = 0
            for item in self._iter_payload_items(data):
                coords = item.get("coordinates", item) or {}

                x = self._to_float(coords.get("x", 0.0))
                y = self._to_float(coords.get("y", 0.0))
                z = self._to_float(coords.get("z", 0.0))
                azm = self._to_float(coords.get("azimuth", 0.0))

                flat.extend([x, y, z, azm])
                count += 1

            self.get_logger().info("\n" + pretty_line())
            self.get_logger().info(f"📡 [UTM Message Received] items={count}")
            self.get_logger().info(pretty_line())
            if count == 1:
                self.get_logger().info(f"Single: [x,y,z,az]= {flat}")
            else:
                self.get_logger().info(f"Batch flattened length={len(flat)}")

            if self.publish_ros and self.pub_utm is not None:
                msg = Float64MultiArray()
                msg.data = flat
                self.pub_utm.publish(msg)
                self.get_logger().info(f"📤 Published /cviss/waypoints/utm: {msg.data}")

            message.ack()

        except Exception as e:
            self.get_logger().error(f"❌ Error occurred in _cb_utm: {e}")
            message.nack()

    def _cb_colmap(self, message: pubsub_v1.subscriber.message.Message):
        try:
            data = json.loads(message.data.decode("utf-8"))

            flat = []
            count = 0
            for item in self._iter_payload_items(data):
                coords = item.get("coordinates", item) or {}
                position = coords.get("position", {})
                rotation = coords.get("rotation", {})

                px = self._to_float(position.get("x", 0.0))
                py = self._to_float(position.get("y", 0.0))
                pz = self._to_float(position.get("z", 0.0))

                # quaternion defaults to identity
                qw = self._to_float(rotation.get("w", 1.0))
                qx = self._to_float(rotation.get("x", 0.0))
                qy = self._to_float(rotation.get("y", 0.0))
                qz = self._to_float(rotation.get("z", 0.0))

                flat.extend([px, py, pz, qw, qx, qy, qz])
                count += 1

            self.get_logger().info("\n" + pretty_line())
            self.get_logger().info(f"📡 [COLMAP Message Received] items={count}")
            self.get_logger().info(pretty_line())
            if count == 1:
                self.get_logger().info(f"Single: [px,py,pz,qw,qx,qy,qz]= {flat}")
            else:
                self.get_logger().info(f"Batch flattened length={len(flat)}")

            if self.publish_ros and self.pub_colmap is not None:
                msg = Float64MultiArray()
                msg.data = flat
                self.pub_colmap.publish(msg)
                self.get_logger().info(f"📤 Published /cviss/waypoints/colmap: {msg.data}")

            message.ack()

        except Exception as e:
            self.get_logger().error(f"❌ Error occurred in _cb_colmap: {e}")
            message.nack()

    # ------------- Param change (optional) -------------

    def _on_param_change(self, params):
        # For simplicity we don’t live-switch mode. Just log attempts.
        for p in params:
            if p.name == "mode":
                self.get_logger().warn("Changing 'mode' at runtime is not supported. Restart the node instead.")
        return rclpy.parameter.SetParametersResult(successful=True)

    # ------------- Shutdown -------------

    def destroy_node(self):
        self._cancel_streaming()
        try:
            self.subscriber.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = PubSubListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
