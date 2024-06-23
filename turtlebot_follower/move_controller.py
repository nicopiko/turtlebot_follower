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
        self.dist_reference = 0.5

    def offset_callback(self,msg:Float32):
        offset = msg.data
        angular_steer = self.kp_steer * offset
        self.steer = angular_steer
        if self.steer == 69.0:
            move_cmd = Twist()
            move_cmd.angular.z = 0.1
            self.vel_pub.publish(move_cmd)
            self.get_logger().info("Robot out of vision. Turning to scan for robot") #DEBUG
            return
        if self.steer != None and self.drive != None:
            move_cmd = Twist()
            move_cmd.angular.z = self.steer
            move_cmd.linear.x = self.drive
            self.vel_pub.publish(move_cmd)

            self.get_logger().info(f"Steer: {self.steer}. Drive: {self.drive}") #DEBUG
    
    def dist_callback(self,msg:Float32):    
        distance = msg.data
        error = distance - self.dist_reference
        self.integral_drive += error * 0.2 #sample time

        drive_cmd = self.kp_drive * error + self.ki_drive * self.integral_drive

        if drive_cmd < -0.215:
            drive_cmd = -0.215
        if drive_cmd > 0.215:
            drive_cmd = 0.215

        self.drive = -drive_cmd
    

def main():
    rclpy.init()
    node = move_reg()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
