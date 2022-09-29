from queue import Empty
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto
from std_srvs.srv import Empty

class State(Enum):
    """ Current state of the system.
        TODO update Determines what the main timer function should be doing on each iteration
    """
    MOVING = auto(),
    STOPPED = auto()
    
class WaypointNode(Node):
    """TODO _summary_

    Args:
        Node (_type_): _description_
    """
    
    def __init__(self):
        super().__init__("waypointnode")
        self.state =  State.STOPPED
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)


        
    def timer_callback(self):
        """
        TODO
        """
        if self.state == State.MOVING:
            self.get_logger().info("Issuing Command!")
    
    def toggle_callback(self, request, response):
        """_summary_
        TODO
        """
        # Switch the state 
        if self.state == State.STOPPED:
            self.state = State.MOVING
        else:
            self.state = State.STOPPED
            self.get_logger().error("Stopping")
        return response

        
def main(args=None):
    """ The main() function. """
    rclpy.init(args=args)
    myWaypoint = WaypointNode()
    rclpy.spin(myWaypoint)
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
