import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class move_reg(Node):
    def __init__(self):
        super().__init__("angular_regulator")
        self.offset_subscriber = self.create_subscription(Float32,"/offset",self.offset_callback,10)
        self.dist_subscriber = self.create_subscription(Float32,"/dist",self.dist_callback,10)
        self.vel_pub = self.create_publisher(Twist,"/cmd_vel",10)

        self.steer = None
        self.drive = None

        self.kp_steer = 1
        
        self.kp_drive = 1
        self.ki_drive = 1
        self.integral_drive = 0

    def offset_callback(self,msg:Float32):
        offset = msg.data
        angular_steer = self.kp_steer * offset
        self.steer = angular_steer
    
    def dist_callback(self,msg:Float32):
        if msg.data == 69.0:
            move_cmd = Twist()
            move_cmd.angular.z = 0.1
            move_cmd.linear.x = 0.0
            self.vel_pub.publish(move_cmd)
        elif self.steer:
            move_cmd = Twist()
            distance = msg.data
            error = distance - 0.8
            self.integral_drive += error * 0.2 #sample time

            drive_cmd = self.kp_drive * error + self.ki_drive * self.integral_drive
            self.drive = -drive_cmd
            
            if self.steer != None and self.drive != None:
                move_cmd = Twist()
                move_cmd.angular.z = self.steer
                move_cmd.linear.x = self.drive
                self.vel_pub.publish(move_cmd)
    

def main():
    rclpy.init()
    node = move_reg()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()