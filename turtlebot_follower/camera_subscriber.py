#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
import cv2
from cv_bridge import CvBridge

class camera_subscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.camera_subscriper = self.create_subscription(Image,"/camera",self.camera_callback,10)
        self.offset_publisher = self.create_publisher(Float32,"/offset",10)
        self.vel_publisher = self.create_publisher(Twist,"cmd_vel",10)
    
    def camera_callback(self,msg:Image):
        bridgeObject = CvBridge()
        offset_msg = Float32()
        frame = bridgeObject.imgmsg_to_cv2(msg,"bgr8")
        frame = cv2.flip(frame,1)
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hsv = cv2.blur(hsv,(18,18))
        lower_yellow = np.array([15, 93, 0])
        upper_yellow = np.array([45, 255, 255])
        thresh = cv2.inRange(hsv,lower_yellow,upper_yellow)

        cont, _ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        try:
            c = cont[0]
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            offset = cX - 160
            offset_msg.data = round(-(offset/160),2)
            self.offset_publisher.publish(offset_msg)
        except:
            offset_msg.data = 69.0
            self.get_logger().info("Robot out of vision")
            self.offset_publisher.publish(offset_msg)

    def camera_callback_debug(self,msg:Image):
        bridgeObject = CvBridge()
        offset_msg = Float32()
        frame = bridgeObject.imgmsg_to_cv2(msg,"bgr8")
        frame = cv2.flip(frame,1)
        hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
        hsv = cv2.blur(hsv,(18,18))
        lower_yellow = np.array([15, 93, 0])
        upper_yellow = np.array([45, 255, 255])
        thresh = cv2.inRange(hsv,lower_yellow,upper_yellow)

        cont, _ = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        try:
            c = cont[0]
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            center = [cX,cY]
            cv2.circle(frame,center,7,(255,255,255),-1)
            offset = cX - 160 
            cv2.putText(frame,f"{offset}",center,cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,255),1)
            offset_msg.data = round(-(offset/160),2)
            self.offset_publisher.publish(offset_msg)
        except:
            offset_msg.data = 69.0
            self.get_logger().info("Robot out of vision")
            self.offset_publisher.publish(offset_msg)
        
        frame = cv2.resize(frame,(640,120))
        thresh = cv2.resize(thresh,(640,120))
        cv2.imshow("frame",frame)
        cv2.imshow("thresh",thresh)
        cv2.imshow("hsv",hsv)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = camera_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
