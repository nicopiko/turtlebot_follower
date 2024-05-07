#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import numpy as np
import cv2
from cv_bridge import CvBridge

class camera_subscriber(Node):
    def __init__(self):
        super().__init__("camera_subscriber")
        self.camera_subscriper = self.create_subscription(Image,"/camera",self.camera_callback,10)
        self.offset_publisher = self.create_publisher(Float32,"/offset",10)
    
    def camera_callback(self,msg:Image):
        bridgeObject = CvBridge()
        offset_msg = Float32()
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

            offset_norm = -(offset/160)

            offset_msg.data = round(offset_norm,2)

            self.offset_publisher.publish(offset_msg)
        except:
            pass

        cv2.imshow("original",frame)
        cv2.imshow("thresh",thresh)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = camera_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()