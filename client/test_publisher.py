#!/usr/bin/env python3
"""Test publisher for WebRTC bridge testing.

Publishes:
- Test images to /camera/image_raw
- Test string messages to /webrtc/string_out

Subscribes:
- Messages from /webrtc/string_in (from remote clients)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import time


class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.string_pub = self.create_publisher(String, '/webrtc/string_out', 10)

        # Subscriber for incoming messages
        self.string_sub = self.create_subscription(
            String, '/webrtc/string_in', self.string_callback, 10)

        # Timers
        self.image_timer = self.create_timer(1.0 / 30.0, self.publish_image)  # 30 FPS
        self.string_timer = self.create_timer(2.0, self.publish_string)  # Every 2 seconds

        self.frame_count = 0
        self.message_count = 0

        self.get_logger().info('Test publisher started')
        self.get_logger().info('Publishing images to /camera/image_raw')
        self.get_logger().info('Publishing strings to /webrtc/string_out')
        self.get_logger().info('Listening on /webrtc/string_in')

    def publish_image(self):
        """Publish a test pattern image."""
        width, height = 640, 480

        # Create a moving gradient pattern
        t = time.time()
        x = np.linspace(0, 1, width)
        y = np.linspace(0, 1, height)
        xx, yy = np.meshgrid(x, y)

        # RGB channels with animation
        r = ((np.sin(xx * 10 + t * 2) + 1) * 127).astype(np.uint8)
        g = ((np.sin(yy * 10 + t * 3) + 1) * 127).astype(np.uint8)
        b = ((np.sin((xx + yy) * 5 + t) + 1) * 127).astype(np.uint8)

        # Stack to create RGB image
        img = np.stack([r, g, b], axis=-1)

        # Add frame counter text area (white rectangle)
        img[10:40, 10:200] = 255

        # Create ROS Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = height
        msg.width = width
        msg.encoding = 'rgb8'
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = img.tobytes()

        self.image_pub.publish(msg)
        self.frame_count += 1

        if self.frame_count % 30 == 0:
            self.get_logger().debug(f'Published frame {self.frame_count}')

    def publish_string(self):
        """Publish a test string message."""
        msg = String()
        msg.data = f'ROS2 message #{self.message_count} at {time.strftime("%H:%M:%S")}'
        self.string_pub.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
        self.message_count += 1

    def string_callback(self, msg):
        """Handle incoming string messages from WebRTC clients."""
        self.get_logger().info(f'Received from WebRTC client: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
