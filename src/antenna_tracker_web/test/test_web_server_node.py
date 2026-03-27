import importlib.util
import os
from pathlib import Path
import socket
from tempfile import TemporaryDirectory
import time
import urllib.request

import rclpy


PACKAGE_DIR = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE_DIR / 'src' / 'web_server_node.py'


def load_web_server_module():
    spec = importlib.util.spec_from_file_location(
        'antenna_tracker_web_server_node',
        MODULE_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


web_server_module = load_web_server_module()


def test_web_server_serves_index_without_changing_cwd():
    original_cwd = os.getcwd()
    with TemporaryDirectory() as temp_dir:
        index_file = Path(temp_dir) / 'index.html'
        index_file.write_text('<html><body>antenna tracker smoke</body></html>', encoding='utf-8')

        rclpy.init(args=['--ros-args', '-p', f'web_root:={temp_dir}', '-p', 'port:=0'])
        node = web_server_module.WebServerNode()
        try:
            response = urllib.request.urlopen(
                f'http://127.0.0.1:{node._server.server_port}/',
                timeout=3.0,
            )
            body = response.read().decode('utf-8')

            assert 'antenna tracker smoke' in body
            assert os.getcwd() == original_cwd
            assert node._thread.is_alive()
        finally:
            node.destroy_node()
            rclpy.shutdown()
            os.chdir(original_cwd)


def test_destroy_node_stops_http_server_thread():
    with TemporaryDirectory() as temp_dir:
        index_file = Path(temp_dir) / 'index.html'
        index_file.write_text('<html><body>shutdown check</body></html>', encoding='utf-8')

        rclpy.init(args=['--ros-args', '-p', f'web_root:={temp_dir}', '-p', 'port:=0'])
        node = web_server_module.WebServerNode()
        port = node._server.server_port

        node.destroy_node()
        rclpy.shutdown()

        assert not node._thread.is_alive()

        time.sleep(0.2)
        try:
            socket.create_connection(('127.0.0.1', port), timeout=1.0)
        except OSError:
            pass
        else:
            raise AssertionError('web server port remained open after destroy_node()')
