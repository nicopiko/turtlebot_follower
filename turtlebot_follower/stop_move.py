import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class stop_move(Node):
    def __init__(self):
        super().__init__("stop_move")
        self.get_logger().info("Stopping all movement")
        self.vel_pub = self.create_publisher(Twist,"/cmd_vel",10)
    
    def stop_move(self):
        move_cmd = Twist()
        move_cmd.angular.z = 0.0
        move_cmd.linear.x = 0.0
        self.vel_pub.publish(move_cmd)
        

def main():
    rclpy.init()
    node = stop_move()
    node.stop_move()
    rclpy.shutdown()

if __name__ == "__main__":
    main()