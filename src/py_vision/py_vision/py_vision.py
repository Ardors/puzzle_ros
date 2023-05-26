from interfaces.srv import IdentifyPiece
from interfaces.srv import LocatePieces
from interfaces.msg import PiecePose

import rclpy
from rclpy.node import Node
from .main import Identification


class PyVision(Node):

    def __init__(self):
        super().__init__('py_vision')
        self.srv = self.create_service(IdentifyPiece, 'py_vision/identify_piece', self.identify_piece_callback)
        self.srv = self.create_service(LocatePieces, 'py_vision/locate_pieces', self.locate_pieces_callback)

    def identify_piece_callback(self, request, response):
        self.get_logger().info('Incoming request')
        
        response.piece.piece_id = Identification()
        response.piece.piece_orientation = 1
        return response
    
    def locate_pieces_callback(self, request, response):
        self.get_logger().info('Incoming request')

        temp = PiecePose()

        temp.piece_x = 0.02
        temp.piece_y = 0.3
        temp.piece_w = 1.2

        response.poses.append(temp)

        temp.piece_x = 0.04
        temp.piece_y = 0.5
        temp.piece_w = 1.3

        response.poses.append(temp)

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = PyVision()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()