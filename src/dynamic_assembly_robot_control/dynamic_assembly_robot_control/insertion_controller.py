import rclpy
from rclpy.node import Node

class InsertionController(Node):

    def __init__(self,node):
        super().__init__('insertion_controller')
        self.node=node

def main(args=None):
    rclpy.init(args=args)
    node = InsertionController()
    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()