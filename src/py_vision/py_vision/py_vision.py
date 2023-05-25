from interfaces.srv import IdentifyPiece

import rclpy
from rclpy.node import Node
from .main import Identification


class PyVision(Node):

    def __init__(self):
        super().__init__('py_vision')
        self.srv = self.create_service(IdentifyPiece, 'py_vision/IdentifyPiece', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        self.get_logger().info('Incoming request')
        
        response.piece.piece_id = Identification()
        response.piece.piece_orientation = 1
        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = PyVision()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()