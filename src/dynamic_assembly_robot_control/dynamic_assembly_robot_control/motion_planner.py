import rclpy
from rclpy.node import Node

class MotionPlanner(Node):

    def __init__(self,node):
        super().__init__('motion_planner')
        self.node=node

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlanner()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()