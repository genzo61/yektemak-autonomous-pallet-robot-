"""Operator panelini (derlenmis React uygulamasi) http ile sunar.

  ros2 run yektemak_fleet dashboard_server   ->  http://localhost:8080
"""

import functools
import http.server
import os
import socketserver

import rclpy
from ament_index_python.packages import get_package_share_directory


def main():
    rclpy.init()
    node = rclpy.create_node("dashboard_server")
    port = node.declare_parameter("port", 8080).value
    root = os.path.join(get_package_share_directory("yektemak_fleet"), "web")
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=root)
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("0.0.0.0", port), handler) as httpd:
        node.get_logger().info(f"Operator paneli: http://localhost:{port}  (klasor: {root})")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
