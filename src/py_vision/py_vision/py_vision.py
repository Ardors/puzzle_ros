from interfaces.srv import IdentifyPiece

import rclpy
from rclpy.node import Node


class PyVision(Node):

    def __init__(self):
        super().__init__('py_vision')
        self.srv = self.create_service(IdentifyPiece, 'py_vision/IdentifyPiece', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        self.get_logger().info('Incoming request')

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = PyVision()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()