#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
from cmath import sqrt


def const(func):
    def wrapper(*args, **kwargs):
        return func(*args, **kwargs)
    return wrapper

class Target_position:
    def __init__(self,x,y) :
        self.x = int (x)
        self.y = int (y)
target_position = Target_position(320,480)

class Middle_position:
    def __init__(self,xmin,xmax,ymin,ymax) :
        self.xmin = int (xmin)
        self.xmax = int (xmax) 
        self.ymin = int (ymin)
        self.ymax = int (ymax) 
middle_position = Middle_position(0,0,0,0)


@const 
def Our_Ball() :
    return "blue"

@const 
def Low_Pass_Filter() :
    return 0.6 

def Distance(xmin,xmax,ymin,ymax,) :
    global target_position 
    return  \
            int (  sqrt( Low_Pass_Filter()*(pow(abs((int (xmin) + int (xmax) )/2 - 320) , 2) + \
                    pow(abs((int (ymin) +int (ymax) )/2 - 480) , 2) ) + \
                    (1-Low_Pass_Filter())*(pow(abs((int(xmin)+int (xmax))/2 - target_position.x) , 2) + \
                    pow(abs((int (ymin)+int (ymax))/2 - target_position.y) , 2))
             ).real )
             
 
class Yolo_Dect:
    
    global middle_position 
    global target_position 
    
    def __init__(self):

        yolov5_path = rospy.get_param('/yolov5_path', '')
        weight_path = rospy.get_param('~weight_pathR', '')
        conf = rospy.get_param('~conf', '0.5')
        iou = rospy.get_param('~iou', '0.35')
        self.model = torch.hub.load(yolov5_path, 'custom',
                                    path=weight_path, source='local')
        
        if (rospy.get_param('/use_cpu', 'false')):
            self.model.cpu()
        else:
            self.model.cuda()

        self.model.conf = conf
        self.model.iou = iou

        image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw') 
        pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')
        # self.camera_frame = rospy.get_param('~camera_frame', '')
        
       
        self.color_image = Image()
        self.getImageStatus = False


        # 设置框的颜色
        self.classes_colors = {}

        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800)
        
        self.position_pub = rospy.Publisher(pub_topic,  BoundingBoxes, queue_size=1)

        self.image_pub = rospy.Publisher('/yolov5/detection_image',  Image, queue_size=1)
        
        # self.image1 = rospy.Publisher("my_image", Image, queue_size=1)

        while (not self.getImageStatus) :
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)



    def image_callback(self, image):

        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        # self.image1.publish(image)

        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

        # xmin    ymin    xmax   ymax  confidence  class    name
        results = self.model(self.color_image)

        boxs = results.pandas().xyxy[0].values
        self.dectshow(self.color_image, boxs)
        cv2.waitKey(3)



    def dectshow(self, org_img, boxs):

        img = org_img.copy()
        sum = 0
        first_ball_flag = 1
        j = 0
        
        for i in boxs:
            sum += 1
        for box in boxs:

            global middle_position 
            global target_position

            boundingBox = BoundingBox()     

            boundingBox.xmin = np.int64(box[0])
            boundingBox.ymin = np.int64(box[1])
            boundingBox.xmax = np.int64(box[2])
            boundingBox.ymax = np.int64(box[3])

            boundingBox.num = np.int16(sum)
            boundingBox.Class = box[-1]
    

            if box[-1] in self.classes_colors.keys():
                color = self.classes_colors[box[-1]]
            else:
                color = np.random.randint(0, 183, 3)
                self.classes_colors[box[-1]] = color

            # 绘制RBG格式
            cv2.rectangle(img, (int(box[0]), int(box[1])),
                          (int(box[2]), int(box[3])), (int(color[0]),int(color[1]), int(color[2])), 2)
            
            if box[1] < 20:
                text_pos_y = box[1] + 30
            else:
                text_pos_y = box[1] - 10
            









            if boundingBox.Class != "basket" :
                cv2.putText(img, box[-1],
                    (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2, cv2.LINE_AA) 


            if boundingBox.Class == "basket" :
                for i in boxs :
                    if (i[-1]  != "basket" and 
                        (i[0] + i[2])/2 > boundingBox.xmin and 
                        (i[0] + i[2])/2 < boundingBox.xmax and
                        (i[1] + i[3])/2 > boundingBox.ymin - 40 and
                        (i[1] + i[3])/2 < boundingBox.ymax                     
                        ):
                        j += 1
                # 判断框里的球的数量
                cv2.putText(img,str(j) , (int(box[0]),int(text_pos_y)+10 ) , 
                            cv2.FONT_HERSHEY_SCRIPT_SIMPLEX , 2 ,(255 ,255 ,255), 2 ,cv2.LINE_AA)
                j = 0

            # first_ball_flag 是第一个球的数据
            if box[-1] == Our_Ball() and first_ball_flag:
                middle_position.xmin = int (box[0] )
                middle_position.xmax = int (box[2] )
                middle_position.ymin = int (box[1] )
                middle_position.ymax = int (box[3] )
                first_ball_flag = 0
            elif box[-1] == Our_Ball() and \
                 first_ball_flag == 0 and \
                Distance(int (box[0]) , int (box[2]), int (box[1]),int (box[3])) < \
                Distance(middle_position.xmin,middle_position.xmax,middle_position.ymin,middle_position.ymax) :
                middle_position.xmin = int (box[0] )
                middle_position.xmax = int (box[2] )
                middle_position.ymin = int (box[1] )
                middle_position.ymax = int (box[3] )
                 
            self.boundingBoxes.bounding_boxes.append(boundingBox)
               
            self.position_pub.publish(self.boundingBoxes)


        # 出循环的判断
        if sum :
            target_position.x = int (( middle_position.xmin + middle_position.xmax )/2 )
            target_position.y = int (( middle_position.ymin + middle_position.ymax )/2 )   
            cv2.putText(img, "target",
                    (target_position.x , target_position.y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
        else:
            target_position.x = 320 
            target_position.y = 480
            middle_position.xmin = 0
            middle_position.xmax = 0
            middle_position.ymin = 0 
            middle_position.ymax = 0
        cv2.imshow('YOLOv5', img)


def main():
    rospy.init_node('yolov5_ros', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":

    main()