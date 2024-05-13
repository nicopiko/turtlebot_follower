import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class angular_reg(Node):
    def __init__(self):
        super().__init__("angular_regulator")
        self.offset_subscriber = self.create_subscription(Float32,"/offset",self.offset_callback,10)
        self.vel_pub = self.create_publisher(Twist,"/cmd_vel",10)

        self.kp = 1

    def offset_callback(self,msg:Float32):
        move_cmd = Twist()
        offset = msg.data
        angular_steer = self.kp * offset
        move_cmd.angular.z = angular_steer
        self.vel_pub.publish(move_cmd)

def main():
    rclpy.init()
    node = angular_reg()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()