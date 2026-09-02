import rclpy
from rclpy.node import Node

class TargetManager(Node):

    def __init__(self,node):
        super().__init__('target_manager')
        self.node=node

def main(args=None):
    rclpy.init(args=args)
    node= TargetManager()
    rclpy.spin(node)



    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()