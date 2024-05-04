import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from math import pi
from std_msgs.msg import Float32
from stop_msgs.msg import PhysicalLocation

# from safety_controller_pkg.visualization_tools import VisualizationTools

#basically our safety controller code but for stopsigns and stoplights

class CityStoppingController(Node):
    def __init__(self):
        super().__init__("wall_follower") #<--- wut is this for again... T-T
        self.create_timer(1.0, self.timer_callback)
        # Declare parameters to make them available for use
        # self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/drive") 
        self.declare_parameter("stop_topic","/drive")
        self.declare_parameter("base_frame", "/base_link")

        #this is so fucking jank but i'm also. tired as fuck
        #when stopping for stop sign, decrement this value
        #when this countdown is done, wait for cooldown to decrement
        #then reset both
        self.ignore_stopsigns = False
        self.stopsign_brake_time = 10 #can adjust i guess, plan to just decrement every time a drive command recieved
        self.stopsign_cooldown = 10
        

        # Fetch constants from the ROS parameter server
        # self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.STOP_TOPIC = self.get_parameter('stop_topic').get_parameter_value().string_value

        self.BASE_FRAME = self.get_parameter('base_frame').get_parameter_value().string_value
       
        
        # Publishers and subscribers
        self.drive_subscriber = self.create_subscription(AckermannDriveStamped, self.DRIVE_TOPIC, self.log_drive_command, 10)
        self.stopsign_subscriber = self.create_subscription(PhysicalLocation, '/relative_stopsign', self.on_stopsign, 10)
        self.stoplight_subscriber = self.create_subscription(PhysicalLocation, '/relative_stoplight', self.on_stoplight, 10)

        self.stop_pub = self.create_publisher(AckermannDriveStamped, self.STOP_TOPIC, 1)
        self.error_pub = self.create_publisher(Float32,"city_stop_error",1)
        

        #max last_drive command at start to make safe initially
        self.last_drive_command = AckermannDriveStamped()
        self.last_drive_command.header.frame_id = self.BASE_FRAME
        self.last_drive_command.drive.speed = 0.0
        self.last_drive_command.drive.steering_angle = 0.0
        self.last_drive_command.header.stamp = self.get_clock().now().to_msg()

        #pre populate stop message
        self.stop_msg = AckermannDriveStamped() #pre populate stop message
        self.stop_msg.header.frame_id = self.BASE_FRAME
        self.stop_msg.drive.speed = 0.0
        self.stop_msg.drive.steering_angle = 0.0

        self.get_logger().info("started up city stopping node")


    def log_drive_command(self,msg):
        """
        Keeps internal record of last drive command issued
        msg:AckermannDriveStamped - drive command issued by wall follower
        return:None sets instance variable
        """
        
        self.last_drive_command = msg

        #decrement stopsign cooldown if currently ignoring stopsigns and cooldown time remaining
        if self.ignore_stopsigns and self.stopsign_cooldown:
            self.stopsign_cooldown-=1
        else: #stopsign cooldown complete
            #reset
            self.ignore_stopsigns = False
            self.stopsign_brake_time = 10
            self.stopsign_cooldown = 10

    def on_stopsign(self,msg):
        """
        Monitors for stop signs and publishes stop command for 2 seconds before moving on

        #TODO: wuh oh just thought of a problem to debug later
        #TODO: also need to actually implement this
        (shit wait yeah it might just keep detecting that stop sign, maybe set an amount of time for stopsign to be ignored?
        but what if there's another stop sign right afterwards that we drive past?)

        msg:PhysicalLocation - relative physical location (x,y) of stop sign from car
        return:None - publishes directions to car if necessary
        """
        
        speed = self.last_drive_command.drive.speed
        t_reac = .05 #frequency msgs are sent at
        u = 1.4 #coefficient of friction used for cars in charts
        grav = 9.8 
        total_stop_d = speed*t_reac + (speed**2)/(2*u*grav)
        threshold = 1 #can adjust until stops consistently at 0.5-1m from light

        y = msg.y_pos

        obj_dist = np.sqrt(x**2 + y**2)

        if obj_dist <= (total_stop_d + threshold) and self.stopsign_brake_time:
            #start braking
            error_msg = Float32()
            error_msg.data = abs(obj_dist-(total_stop_d+threshold))/(total_stop_d+threshold)
            self.error_pub.publish(error_msg)
            self.stop_msg.header.stamp = self.get_clock().now().to_msg()
            self.stop_msg.drive.steering_angle = self.last_drive_command.drive.steering_angle

            #TODO: find way to make it come to a full stop and then continue on without reacting to same stop sign
            self.stop_pub.publish(self.stop_msg)
            self.stopsign_brake_time -= 1 #stop for about one second (callback runs at 10Hz)
        elif obj_dist <= (total_stop_d + threshold) and not self.stopsign_brake_time and self.stopsign_cooldown: #within range and done braking, start cooldown
            self.ignore_stopsigns=True #start cooldown
            

    def on_stoplight(self,msg):
        """
        Monitors for red stoplights within 1m of car and publishes stop command until red stoplight no longer detected
        (relatively easier than stopsigns ngl)

        msg:PhysicalLocation - relative physical location (x,y) of red stoplight from car
        return:None - publishes directions to car if necessary
        """

        speed = self.last_drive_command.drive.speed
        t_reac = .05 #frequency msgs are sent at
        u = 1.4 #coefficient of friction used for cars in charts
        grav = 9.8 
        total_stop_d = speed*t_reac + (speed**2)/(2*u*grav)
        threshold = 1 #can adjust threshold until car consistently stops within 0.5-1m

        x = msg.x_pos
        y = msg.y_pos
        
        obj_dist = np.sqrt(x**2 + y**2)

        if obj_dist <= (total_stop_d + threshold):
            error_msg = Float32()
            error_msg.data = abs(obj_dist-(total_stop_d+threshold))/(total_stop_d+threshold)
            self.error_pub.publish(error_msg)
            self.stop_msg.header.stamp = self.get_clock().now().to_msg()
            self.stop_msg.drive.steering_angle = self.last_drive_command.drive.steering_angle
            self.stop_pub.publish(self.stop_msg)


def main():
    rclpy.init()
    city_stopping_controller = CityStoppingController()
    try:
        rclpy.spin(city_stopping_controller)
    except KeyboardInterrupt:
        pass
    city_stopping_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
