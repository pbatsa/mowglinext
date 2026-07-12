#!/usr/bin/env python3
"""Publish onboard computer health diagnostics.

This node is intended to run on the mower/onboard role. In a distributed setup
the GUI often runs on a remote computer, so GUI-local thermal reads report the
wrong Pi. Publishing the onboard temperature on /diagnostics gives the remote
GUI an authoritative mower-side value.
"""

from __future__ import annotations

import json
import socket
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node


def key_value(key: str, value: object) -> KeyValue:
    msg = KeyValue()
    msg.key = key
    msg.value = str(value)
    return msg


class HostHealthNode(Node):
    def __init__(self) -> None:
        super().__init__("host_health_node")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("temperature_path", "/sys/class/thermal/thermal_zone0/temp")
        self.declare_parameter("temperature_warn_c", 70.0)
        self.declare_parameter("temperature_error_c", 80.0)
        self.declare_parameter("docker_socket_path", "/var/run/docker.sock")

        publish_rate = float(self.get_parameter("publish_rate").value)
        if publish_rate <= 0.0:
            self.get_logger().warning("publish_rate must be positive; using 1.0 Hz")
            publish_rate = 1.0

        self.publisher = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_health)
        self.hostname = socket.gethostname()

    def read_temperature_c(self) -> float | None:
        path = Path(str(self.get_parameter("temperature_path").value))
        try:
            raw = path.read_text(encoding="utf-8").strip()
            return float(raw) / 1000.0
        except (OSError, ValueError) as exc:
            self.get_logger().debug(f"Unable to read CPU temperature from {path}: {exc}")
            return None

    def read_docker_containers(self) -> list[dict[str, object]]:
        socket_path = Path(str(self.get_parameter("docker_socket_path").value))
        if not socket_path.exists():
            return []

        request = (
            "GET /containers/json?all=1 HTTP/1.1\r\n"
            "Host: docker\r\n"
            "Connection: close\r\n"
            "\r\n"
        ).encode("ascii")

        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(1.0)
                client.connect(str(socket_path))
                client.sendall(request)
                chunks: list[bytes] = []
                while True:
                    chunk = client.recv(65536)
                    if not chunk:
                        break
                    chunks.append(chunk)
        except OSError as exc:
            self.get_logger().debug(f"Unable to query Docker socket {socket_path}: {exc}")
            return []

        response = b"".join(chunks)
        _, separator, body = response.partition(b"\r\n\r\n")
        if not separator:
            return []

        try:
            parsed = json.loads(body.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            self.get_logger().debug(f"Unable to parse Docker container response: {exc}")
            return []

        return parsed if isinstance(parsed, list) else []

    @staticmethod
    def container_name(container: dict[str, object]) -> str:
        names = container.get("Names")
        if isinstance(names, list) and names:
            name = str(names[0]).lstrip("/")
            if name:
                return name
        return str(container.get("Id", "unknown"))[:12]

    @staticmethod
    def container_started_at(container: dict[str, object]) -> str:
        created = container.get("Created")
        if isinstance(created, (int, float)):
            return datetime.fromtimestamp(created, tz=timezone.utc).isoformat().replace("+00:00", "Z")
        return ""

    def build_host_status(self) -> DiagnosticStatus:
        temp_c = self.read_temperature_c()
        warn_c = float(self.get_parameter("temperature_warn_c").value)
        error_c = float(self.get_parameter("temperature_error_c").value)

        status = DiagnosticStatus()
        status.name = "Onboard Computer"
        status.hardware_id = self.hostname

        if temp_c is None:
            status.level = DiagnosticStatus.STALE
            status.message = "Onboard CPU temperature unavailable"
        elif temp_c >= error_c:
            status.level = DiagnosticStatus.ERROR
            status.message = "Onboard CPU temperature critical"
        elif temp_c >= warn_c:
            status.level = DiagnosticStatus.WARN
            status.message = "Onboard CPU temperature elevated"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "Onboard CPU temperature OK"

        status.values = [
            key_value("role", "onboard"),
            key_value("host", self.hostname),
            key_value("cpu_temperature_c", f"{temp_c:.1f}" if temp_c is not None else "nan"),
            key_value("temperature_warn_c", f"{warn_c:.1f}"),
            key_value("temperature_error_c", f"{error_c:.1f}"),
            key_value("source", str(self.get_parameter("temperature_path").value)),
        ]
        return status

    def build_container_statuses(self) -> list[DiagnosticStatus]:
        statuses: list[DiagnosticStatus] = []
        for container in self.read_docker_containers():
            if not isinstance(container, dict):
                continue

            name = self.container_name(container)
            state = str(container.get("State", "unknown"))
            state_text = str(container.get("Status", ""))

            status = DiagnosticStatus()
            status.name = f"Onboard Container/{name}"
            status.hardware_id = self.hostname
            status.level = DiagnosticStatus.OK if state == "running" else DiagnosticStatus.ERROR
            status.message = f"{name}: {state_text or state}"
            status.values = [
                key_value("role", "onboard"),
                key_value("host", self.hostname),
                key_value("name", name),
                key_value("state", state),
                key_value("status", state_text),
                key_value("started_at", self.container_started_at(container)),
            ]
            statuses.append(status)
        return statuses

    def publish_health(self) -> None:
        statuses = [self.build_host_status()]
        statuses.extend(self.build_container_statuses())

        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = statuses
        self.publisher.publish(array)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HostHealthNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
