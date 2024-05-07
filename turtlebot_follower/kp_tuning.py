#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge
import time as t

class camera_subscriber(Node):
    def __init__(self):
        super().__init__("kp_tuning")
        self.camera_subscriper = self.create_subscription(Image,"/camera",self.camera_callback,10)
        self.vel_pub = self.create_publisher(Twist,"/cmd_vel",10)

        self.current_step = 0
        self.offset_array = []
        self.kp = 0.1
        

    def camera_callback(self,msg:Image):
        bridgeObject = CvBridge()
        frame = bridgeObject.imgmsg_to_cv2(msg)
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hsv = cv2.blur(hsv,(9,9))
        lower_red = np.array([161,155,84])
        upper_red = np.array([179,255,255])
        thresh = cv2.inRange(hsv,lower_red,upper_red)

        cont, _ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        try:
            c = cont[0]
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            center = [cX,cY]

            cv2.circle(frame, center, 7, (255, 255, 255), -1)
            offset = 160 - cX

            offset_norm = round(-(offset/160),2)
            self.offset_array.append(offset_norm)
            if len(self.offset_array) > 20:
                self.offset_array.pop(0)
        except:
            pass

        cv2.imshow("original",frame)
        cv2.imshow("thresh",thresh)
        cv2.waitKey(1)


        if self.current_step == 0:
            self.get_logger().info(f"Centering")
            self.current_step = 1

        if self.current_step == 1:
            move_cmd = Twist()
            if all(v == 0 for v in self.offset_array) and len(self.offset_array) > 19:
                self.current_step = 2
                move_cmd.angular.z = 0.0
                self.vel_pub.publish(move_cmd)
            else:
                angular_steer = 1.82 * offset_norm
                move_cmd.angular.z = angular_steer
                self.vel_pub.publish(move_cmd)
                print("waiting for all 20 previous measurements to be 0")
                
        
        if self.current_step == 2:
            self.get_logger().info("Simulating step response")
            self.current_step = 3
        
        if self.current_step == 3:
            move_cmd = Twist()
            if all(v == -0.6 for v in self.offset_array) and len(self.offset_array) > 19:
                self.current_step = 0
                move_cmd.angular.z = 0.0
                self.vel_pub.publish(move_cmd)
            else:
                angular_steer = 1.82 * (offset_norm + 0.6)
                move_cmd.angular.z = angular_steer
                self.vel_pub.publish(move_cmd)
                print(self.offset_array)
        
        if self.current_step == 4:
            self.get_logger().info(f"Offset: {offset_norm}")
def main():
    rclpy.init()
    node = camera_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()