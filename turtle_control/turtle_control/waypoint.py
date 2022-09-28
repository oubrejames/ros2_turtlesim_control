import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

# TODO site ros example

class WaypointNode(Node):
    """TODO _summary_

    Args:
        Node (_type_): _description_
    """
    
    def __init__(self):
        super().__init__("waypointnode")
        self.timer = self.create_timer(0.01, self.timer_callback)
        
    def timer_callback(self):
        self.get_logger().debug("Issuing Command!")
    
def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    myWaypoint = WaypointNode()
    rclpy.spin(myWaypoint)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
