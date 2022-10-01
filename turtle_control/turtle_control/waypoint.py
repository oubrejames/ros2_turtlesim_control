from queue import Empty
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
from turtlesim.srv import Spawn, Kill, TeleportAbsolute


class State(Enum):
    """ Current state of the system.
        TODO update Determines what the main timer function should be doing on each iteration
    """
    MOVING = auto(),
    STOPPED = auto(),
    TELEPORT = auto()
    
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
        self.load = self.create_service(Waypoints, "load", self.load_callback)
        self.reset = self.create_client(Empty, "reset")
        self.teleport = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute")
        
        if not self.teleport.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "teleport_absolute" service to become available')
            
    def load_callback(self, waypoint_list, response):
        """TODO
        
        """
        # Reset turtle
        #print("Reseting Turtle")
        #self.reset_future = self.reset.call_async(Empty.Request())
        
        # Draw X at each waypoint
        
        # Move turtle to first waypoint
        # Turtle shouldn't move until toggle is called
        print(waypoint_list)
        for i in range( len(waypoint_list.mixer)):
            xx = waypoint_list.mixer[i].x
            yy = waypoint_list.mixer[i].y
            tt = waypoint_list.mixer[i].theta
            self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = xx , y = yy, theta = tt))
            self.state = State.TELEPORT

        # Computes straight line distance of waypoints 
        return response
    
    def timer_callback(self):
        """
        TODO
        """
        if self.state == State.MOVING:
            self.get_logger().info("Issuing Command!")

        if self.state == State.TELEPORT:
            if self.teleport_future.done():
                #self.teleport_future2 = self.teleport.call_async(TeleportAbsolute.Request(x = 0.0 , y = 7.0 , theta = 2.0))
                self.state = State.STOPPED    
                
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
