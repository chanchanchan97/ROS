#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError

class vehicleDetector:
    def __init__(self):
        # 创建cv_bridge
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("cv_bridge_image", Image, queue_size=1)
        
        video = rospy.get_param("~video_vehicle", "")
        cascade_vehicle = rospy.get_param("~cascade_vehicle", "")
        
        # 使用级联表初始化haar特征检测器
        self.cascade_vehicle = cv2.CascadeClassifier(cascade_vehicle)
        
        # 设置级联表的参数，优化人脸识别，可以在launch文件中重新配置
        self.haar_scaleFactor  = rospy.get_param("~haar_scaleFactor", 1.2)
        self.haar_minNeighbors = rospy.get_param("~haar_minNeighbors", 2)
        self.haar_minSize      = rospy.get_param("~haar_minSize", 20)
        self.haar_maxSize      = rospy.get_param("~haar_maxSize", 55)
        self.color = (50, 255, 50)
        
        cap = cv2.VideoCapture(video)
        
        while(cap.isOpened()):
            ret, cv_image = cap.read()
            cv_image_compressed = cv2.resize(cv_image, (960,540))
            frame = cv_image_compressed
			
            if ret==True:
                # 创建灰度图像
                grey_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # 创建平衡直方图，减少光线影响
                grey_image = cv2.equalizeHist(grey_image)

                vehicle_result = self.detect_vehicle(grey_image)
        		
                if len(vehicle_result)>0:
                    for vehicle in vehicle_result:
                        x, y, w, h = vehicle
                        cv2.rectangle(cv_image_compressed, (x, y), (x+w, y+h), self.color, 2)

                cv2.imshow('frame',cv_image_compressed)
                if cv2.waitKey(25) & 0xFF == ord('q'):
                    break
            else:
                break

    def detect_vehicle(self, input_image):
        # 首先匹配正面人脸的模型
        if self.cascade_vehicle:
            vehicles = self.cascade_vehicle.detectMultiScale(input_image, 
                    self.haar_scaleFactor, 
                    self.haar_minNeighbors, 
                    cv2.CASCADE_SCALE_IMAGE, 
                    (self.haar_minSize, self.haar_maxSize))
        
        return vehicles

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("vehicle_detector")
        vehicleDetector()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down face detector node."
        cv2.destroyAllWindows()
