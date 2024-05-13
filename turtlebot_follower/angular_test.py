import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time as t


class angular_reg(Node):
    def __init__(self):
        super().__init__("angular_regulator")
        self.offset_subscriber = self.create_subscription(Float32,"/offset",self.offset_callback,10)
        self.vel_pub = self.create_publisher(Twist,"/cmd_vel",10)

        self.kp = 1.82

        self.state = 0

        self.error_array = [] #Last 20 error readings

        self.error_data = [] #Data readings from step response

    def offset_callback(self,msg:Float32):
        if self.state == 0:
            #INIT
            self.offset_data = []
            self.error_array = []
            self.get_logger().info(f"Angular test. Turning to center")
            move_cmd = Twist()
            move_cmd.angular.z = 0.0
            self.vel_pub.publish(move_cmd)
            self.state = 1
        
        if self.state == 1:
            #CENTER
            self.regulate_angle(0,msg.data)

        if self.state == 2:
            self.get_logger().info("Turning to offset 0.7")
            self.state += 1

        if self.state == 3:
            #GO TO 0.7 OFFSET
            self.regulate_angle(0.7,msg.data)
        
        if self.state == 4:
            self.get_logger().info("Turning to center, and logging data")
            self.state += 1
        
        if self.state == 5:
            #CENTER AGAIN, AND LOG DATA
            self.log_data(0,msg.data)
            self.regulate_angle(0,msg.data)
        
        if self.state == 6:
            self.get_logger().info(f"Test done. Error array: {self.error_data}")
            file = open("/home/nicolai/Documents/turtlebot/angular_test.txt","w")
            file.write(str(self.error_data))
            file.close()
            self.state = 7

        if self.state == 7:
            move_cmd = Twist()
            move_cmd.angular.z = 0.0
            self.vel_pub.publish(move_cmd)

    def regulate_angle(self,goal,offset):
        move_cmd = Twist()
        error = offset - goal
        angular_steer = self.kp * error
        
        error_rounded = round(error,1)
        self.error_array.append(error_rounded)
        if len(self.error_array) > 100:
            self.error_array.pop(0)

        move_cmd.angular.z = angular_steer
        self.vel_pub.publish(move_cmd)

        if all(v == 0 for v in self.error_array) and len(self.error_array) > 99:
            self.state += 1
        else:
            pass

    def log_data(self,goal,offset):
        error = offset - goal
        self.error_data.append(error)
        pass

def main():
    rclpy.init()
    node = angular_reg()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()