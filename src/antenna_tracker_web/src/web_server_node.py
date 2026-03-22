#!/usr/bin/env python3
"""
Lightweight HTTP server node to serve the web dashboard static files.
Serves from the `web/` directory on port 8080 by default.
"""
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from ament_index_python.packages import get_package_share_directory
import http.server
import threading
import os


class WebServerNode(Node):
    def __init__(self):
        super().__init__('gcs_web_server')
        self.declare_parameter('web_root', '', ParameterDescriptor(description='Path to web static files'))
        self.declare_parameter('port', 8080, ParameterDescriptor(description='HTTP server port'))

        web_root = self.get_parameter('web_root').value or \
                   os.path.join(get_package_share_directory('antenna_tracker_web'), 'web')
        port = self.get_parameter('port').value

        web_root = os.path.realpath(web_root)
        self.get_logger().info(f'GCS Web server: http://0.0.0.0:{port}')
        self.get_logger().info(f'  Serving: {web_root}')

        Handler = http.server.SimpleHTTPRequestHandler
        os.chdir(web_root)
        self._server = http.server.HTTPServer(('', port), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def destroy_node(self):
        self._server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
