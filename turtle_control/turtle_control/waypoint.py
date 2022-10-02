from queue import Empty
import turtle
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from enum import Enum, auto
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
from turtlesim.srv import Spawn, Kill, TeleportAbsolute, SetPen
from turtlesim.msg import Pose
from time import sleep
from geometry_msgs.msg import Twist, Vector3
import math

class State(Enum):
    """ Current state of the system.
        TODO update Determines what the main timer function should be doing on each iteration
    """
    MOVING = auto(),
    STOPPED = auto(),
    TELEPORT = auto(),
    RESET = auto(),
    SETPEN = auto(),
    LIMBO = auto()
                
class WaypointNode(Node):
    """TODO _summary_

    Args:
        Node (_type_): _description_
    """
    
    def __init__(self):
        super().__init__("waypointnode")
        self.state =  State.STOPPED
        self.declare_parameter("frequency", 100.0,
                               ParameterDescriptor(description="The frequency to send commands to the turtle"))
        self.frequency  = self.get_parameter("frequency").get_parameter_value().double_value
        self.declare_parameter("tolerance", 0.2,
                               ParameterDescriptor(description="Toelrance of turtle to waypoint"))
        self.tolerance  = self.get_parameter("tolerance").get_parameter_value().double_value
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)
        self.toggle = self.create_service(Empty, "toggle", self.toggle_callback)
        self.load = self.create_service(Waypoints, "load", self.load_callback)
        self.reset = self.create_client(Empty, "reset")
        self.teleport = self.create_client(TeleportAbsolute, "turtle1/teleport_absolute")
        self.X_count = 0 # Count of the number of x's (waypoints)
        self.X_point_count = 0 # Which point of the x are we at? (center or one of the four corners)
        self.X_done = False # Flag for if x has been totally drawn
        self.all_X_done = False # Flag for if done drawing all x's
        self.set_pen = self.create_client(SetPen, "turtle1/set_pen")
        self.pub_vel = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.way_count = 0 # Counter to keep track of what waypoint to move to
        self.sub = self.create_subscription(Pose, "turtle1/pose", self.listener_callback, 10)
        self.spin_count = 0 # Keep track of spin number
               
    def listener_callback(self, msg):
        """Get turtle pose
        """
        self.turtle_pose = msg
        #print(self.turtle_pose)
        
    def move_to_waypoint(self):
        """
        Moves turtle to the next waypoint
        """
        if self.state == State.MOVING:
            self.set_pen_future = self.set_pen.call_async(SetPen.Request(r = 3, g = 252, b = 169, width = 3, off=0))
            y= self.waypoint_l.mixer[self.way_count].y - self.turtle_pose.y
            x= self.waypoint_l.mixer[self.way_count].x - self.turtle_pose.x
            if self.spin_count % 2 == 0: # Alternate updating ang vs lin to speed up the process
                # Move forward in x 
                move_turtle_lin = Twist(linear = Vector3(x = 1.0, y = 0.0 ,z =0.0), 
                                        angular = Vector3(x = 0.0, y = 0.0, z = 0.0))
                self.pub_vel.publish(move_turtle_lin) 
            else:
                # Calculate angle to waypoint (phi)
                phi = math.atan2(y,x)
                
                # Turn towards waypoint
                print("PHI - theta = ", phi - self.turtle_pose.theta)
          
                if abs(phi - self.turtle_pose.theta) < 3.14:
                            
                    if self.turtle_pose.theta < phi:
                        move_turtle_ang = Twist(linear = Vector3(x = 0.0, y = 0.0 ,z =0.0), 
                                                angular = Vector3(x = 0.0, y = 0.0, z = 2.5))
                        self.pub_vel.publish(move_turtle_ang)
                    if self.turtle_pose.theta > phi:
                        move_turtle_ang = Twist(linear = Vector3(x = 0.0, y = 0.0 ,z =0.0), 
                                                angular = Vector3(x = 0.0, y = 0.0, z = -2.5))
                        self.pub_vel.publish(move_turtle_ang) 
                else:
                    if self.turtle_pose.theta < phi:
                        move_turtle_ang = Twist(linear = Vector3(x = 0.0, y = 0.0 ,z =0.0), 
                                                angular = Vector3(x = 0.0, y = 0.0, z = -2.5))
                        self.pub_vel.publish(move_turtle_ang)
                    if self.turtle_pose.theta > phi:
                        move_turtle_ang = Twist(linear = Vector3(x = 0.0, y = 0.0 ,z =0.0), 
                                                angular = Vector3(x = 0.0, y = 0.0, z = 2.5))
                        self.pub_vel.publish(move_turtle_ang)                       
                

    def load_callback(self, waypoint_list, response):
        """TODO
        
        """
        # Reset turtle
        if self.state == State.STOPPED:
            #print("STATE = STOPPED")
            self.reset_turtle()
            self.waypoint_l = waypoint_list
            self.response_l = response
        else:
            self.state = State.TELEPORT
            self.drawX()
                             
        # Move turtle to first waypoint
        # Turtle shouldn't move until toggle is called
        if self.state == State.MOVING:
            self.move_to_waypoint()
            
        
        
        # Computes straight line distance of waypoints 
        
        return response
                    
    def reset_turtle(self):
        #print("Reseting Turtle")
        # Below lines are to make sure we can reset the turtle and the point and x count start over
        self.all_X_done = False
        self.X_done = False
        self.X_count = 0
        self.X_point_count = 0
        
        self.reset_future = self.reset.call_async(Empty.Request())
        
        self.state = State.RESET
        #print("STATE = RESET")
          
    def drawX(self):
        """Takes in a set of waypoints from the load service and draws an x at each.

        Args:
            points (turtlesim/Pose[]): List of waypoints to draw an X at
        """
        #for i in range( len(point.mixer)):
        # Draw first line
        i = self.X_count # The count of each x instance
        point = self.waypoint_l
        #print("Point", i)#point.mixer[i])
        #print("Point count", self.X_point_count)
        # Turn off pen
        #print("Current X: ", self.X_count)
        #print("Current X Point: ", self.X_point_count)

        #Go to center of x
        if not self.all_X_done:
            if self.X_point_count == 0:
                xx = point.mixer[i].x
                yy = point.mixer[i].y
                self.state = State.TELEPORT
                self.set_pen_future = self.set_pen.call_async(SetPen.Request(r=255,width=5,off=1))
                self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = xx , y = yy, theta = 0.0))
                self.state = State.TELEPORT
                
                
            if self.X_point_count == 1:
                draw_x_1 = point.mixer[i].x - 0.25
                draw_y_1 = point.mixer[i].y - 0.25
                self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = draw_x_1 , y = draw_y_1, theta = 0.0))
                
                
            if self.X_point_count == 2:            
                # Turn on pen
                self.set_pen_future = self.set_pen.call_async(SetPen.Request(r=255,width=5,off=0))
                draw_x_2 = point.mixer[i].x + 0.25
                draw_y_2 = point.mixer[i].y + 0.25    
                self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = draw_x_2 , y = draw_y_2, theta = 0.0))
                
                
            if self.X_point_count == 3:     
                # Draw first line
                # Turn off pen
                self.set_pen_future = self.set_pen.call_async(SetPen.Request(r=255,width=5,off=1))
                draw_x_3 = point.mixer[i].x - 0.25
                draw_y_3 = point.mixer[i].y + 0.25
                self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = draw_x_3 , y = draw_y_3, theta = 0.0))
                
            if self.X_point_count == 4:     
                # Turn on pen
                self.set_pen_future = self.set_pen.call_async(SetPen.Request(r=255,width=5,off=0))
                draw_x_4 = point.mixer[i].x + 0.25
                draw_y_4 = point.mixer[i].y - 0.25    
                self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = draw_x_4 , y = draw_y_4, theta = 0.0))
                # Teleport to firt waypoint after making x's
                self.X_done = True
                if self.X_done == True:
                    xx = point.mixer[0].x
                    yy = point.mixer[0].y
                    self.set_pen_future = self.set_pen.call_async(SetPen.Request(r=255,width=5,off=1))
                    self.teleport_future = self.teleport.call_async(TeleportAbsolute.Request(x = xx , y = yy, theta = 0.0))
                   
    def timer_callback(self):
        """
        TODO
        """
        # print("Actual state = ", self.state)
        # print("Timer Callback")
        self.spin_count += 1
        if self.state == State.MOVING:
            #self.get_logger().info("Issuing Command!")
            self.move_to_waypoint()
            if (abs(self.turtle_pose.x - self.waypoint_l.mixer[self.way_count].x) < self.tolerance and 
                abs(self.turtle_pose.y - self.waypoint_l.mixer[self.way_count].y) < self.tolerance):
                self.way_count += 1 
                print("weeee")

        if self.state == State.TELEPORT:
            #print("State = TELEPORT")
            if self.teleport_future.done():
                #print(f"Teleport {self.X_count} done")
                if not self.X_done:
                    #print(self.X_point_count)
                    self.X_point_count += 1
                    self.drawX()   
                elif self.X_count == len(self.waypoint_l.mixer)-1:
                    # print("LENGTH", len(self.waypoint_l.mixer))
                    self.all_X_done = True
                    self.X_count = 0
                    self.state = State.STOPPED
                    #print("DONE Teleporting")
                else:
                    self.X_count += 1
                    self.X_point_count = 0
                    self.drawX()
                    self.X_done = False
                
        if self.state == State.RESET:
            #print("State = reset")
            if self.reset_future.done():
                print("Reset future done")
                self.load_callback(self.waypoint_l, self.response_l)
                self.state = State.TELEPORT
                
                
        #print("STATE =", self.state)
                 
    def toggle_callback(self, request, response):
        """_summary_
        TODO
        """
        # Switch the state 
        if self.state == State.STOPPED:
            self.state = State.MOVING
            
        elif self.state is not State.MOVING:
            print("Hol' on, I'm doing other stuff")
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
