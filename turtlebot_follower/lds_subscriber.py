import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
import numpy as np
import matplotlib.pyplot as plt
from math import radians
from math import pi
from geometry_msgs.msg import Twist

class lidar_gui(Node):
    def __init__(self):
        super().__init__('lidar_gui')
        self.get_logger().info("Opening LiDAR GUI")
        self.center = 0

        plt.ion()
        self.fig,self.axs = plt.subplots(subplot_kw={'projection': 'polar'})
        angle = list(range(360))
        self.angle_rad = [radians(a) for a in angle]

        self.lidar_subscriber = self.create_subscription(LaserScan,"/scan",self.lidar_callback_debug,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.center_subscriber = self.create_subscription(Float32,"/offset",self.center_callback,10)

        self.distance_pub = self.create_publisher(Float32,"/dist",10)

        self.kp = 1

    def lidar_callback_debug(self,msg:LaserScan):
        lds = np.array(msg.ranges)
        self.axs.clear()
        self.axs.set_theta_offset(1.5*pi)
        
        #Show whole scan
        self.axs.plot(self.angle_rad,lds,"g.")

        
        #color part where leader is found
        dist_msg = Float32()
        center_deg = int((self.center - (-1)) / (1 - (-1)) * (205 - 145) + 145)
        lds_new = np.array(lds)
        for deg in range(len(lds)):
            if deg != center_deg:
                lds_new[deg] = 4
        distance = lds[center_deg]
        dist_msg.data = float(distance)
        self.distance_pub.publish(dist_msg)
        self.axs.plot(self.angle_rad,lds_new,"r.")

        #draw graph
        self.axs.set_ylim(0,3)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def lidar_callback(self,msg:LaserScan):
        dist_msg = Float32()
        center_deg = int((self.center - (-1)) / (1 - (-1)) * (205 - 145) + 145)
        lds = np.array(msg.ranges)
        distance = lds[center_deg]
        dist_msg.data = float(distance)
        self.distance_pub.publish(dist_msg)
  
    def center_callback(self,msg:Float32):
        self.center = msg.data

        

def main():
    rclpy.init()
    node = lidar_gui()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()