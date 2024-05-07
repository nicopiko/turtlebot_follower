import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time as t


class test(Node):
    def __init__(self):
        super().__init__("Straight_test")

        self.state = 0
        self.start_time = None

        self.array = []

        self.vel_pub = self.create_publisher(Twist,"/cmd_vel",10)
        self.lidar_subscriber = self.create_subscription(LaserScan,"/scan",self.lidar_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))



    def lidar_callback(self,msg:LaserScan):
        if self.state == 0:
            move_cmd = Twist()
            lds = np.array(msg.ranges)

            self.start_time = t.monotonic()

            self.array.append([0,lds[180]])

            self.get_logger().info("Setting speed to 0.15...")
            move_cmd.linear.x = -0.15
            self.vel_pub.publish(move_cmd)
            self.state = 1
        
        if self.state == 1:
            move_cmd = Twist()
            lds = np.array(msg.ranges)

            new_time = t.monotonic() - self.start_time

            self.array.append([new_time,lds[180]])

            if lds[180] < 0.2:
                move_cmd.linear.x = -0.0
                self.vel_pub.publish(move_cmd)
                self.state = 2

        if self.state == 2:
            self.get_logger().info(f"Test done")
            for reading in self.array:
                print(f"{reading[0]},{reading[1]}")
            self.state = 3
        
        if self.state == 3:
            pass

def main():
    rclpy.init()
    node = test()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()