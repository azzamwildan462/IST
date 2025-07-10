#!/usr/bin/env python3
import os
import threading

import rclpy
from rclpy.node import Node
from flask import Flask, request, abort

class UploadServerNode(Node):
    def __init__(self):
        super().__init__('upload_server_node')

        # Where to write incoming files
        self.declare_parameter("config_dir", "")
        self.upload_dir = self.get_parameter("config_dir").get_parameter_value().string_value
        os.makedirs(self.upload_dir, exist_ok=True)

        # Build Flask app
        app = Flask(__name__)

        @app.route('/uploadA', methods=['POST'])
        def upload_a():
            f = request.files.get('fileA')
            if not f:
                abort(400, 'Missing form field "fileA"')
            dest = os.path.join(self.upload_dir, 'waypoint.csv')
            f.save(dest)
            return f'✅ waypoint.csv saved to {dest}', 200

        @app.route('/uploadB', methods=['POST'])
        def upload_b():
            f = request.files.get('fileB')
            if not f:
                abort(400, 'Missing form field "fileB"')
            dest = os.path.join(self.upload_dir, 'terminal.csv')
            f.save(dest)
            return f'✅ terminal.csv saved to {dest}', 200

        # Start Flask in a daemon thread so it doesn't block rclpy.spin()
        thread = threading.Thread(
            target=lambda: app.run(host='0.0.0.0', port=6273, use_reloader=False),
            daemon=True
        )
        thread.start()
        self.get_logger().info('Flask upload endpoints running on port 6273')

def main(args=None):
    rclpy.init(args=args)
    node = UploadServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down ROS2 node.')
        rclpy.shutdown()

if __name__ == '__main__':
    main()
