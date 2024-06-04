#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
class Yolo_Dect:
    # 初始化 权重模型初始化
    # 发布者消息初始化
    def __init__(self):

        # 权重模型导入 及初始化
        yolov5_path = rospy.get_param('/yolov5_path', '')
        # 这个需要改
        weight_pathK = rospy.get_param('~weight_pathR', '')
        conf = rospy.get_param('~conf', '0.5')
        iou = rospy.get_param('~iou', '0.35')
        self.model = torch.hub.load(yolov5_path, 'custom', path=weight_pathK, source='local')

        if (rospy.get_param('/use_cpu', 'false')):
            self.model.cpu()
        else:
            self.model.cuda()

        # 给模型传入阈值参数
        self.model.conf = conf
        self.model.iou = iou

        # 图像数据的处理接收与发布
        image_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
        pub_topic = rospy.get_param('~pub_topic', '/yolov5/BoundingBoxes')
        self.color_image = Image()
        self.getImageStatus = False
        # 这个是矩形的颜色设定
        self.classes_colors = {}
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800)  
        self.position_pub = rospy.Publisher(pub_topic,  BoundingBoxes, queue_size=1)
        self.image_pub = rospy.Publisher('/yolov5/detection_image',  Image, queue_size=1)
        while (not self.getImageStatus):
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)
    # 对图像进行处理 并通过模型进行数值预测
    def image_callback(self, image):
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        # 重塑ROS类型 这里把color_image转化成RGB的形式  从ros图像到cv图像
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        # 用cv 进行 图像颜色处理       
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
        # 开始执行 模型是需要cv形式的图像的
        results = self.model(self.color_image)
        # 使用 YOLOv5 模型检测目标后得到的边界框（bounding boxes）的表示形式之一
        boxs = results.pandas().xyxy[0].values
        self.dectshow(self.color_image, boxs)
        cv2.waitKey(3)

    # 传入RBG的图像 目标的一些数值
    def dectshow(self, img, boxs):
        count = 0
        for box in boxs: 
            boundingBox = BoundingBox()
            boundingBox.probability =np.float64(box[4])
            boundingBox.xmin = np.int64(box[0])
            boundingBox.ymin = np.int64(box[1])
            boundingBox.xmax = np.int64(box[2])
            boundingBox.ymax = np.int64(box[3])
            boundingBox.Class = box[-1]
            # 为每种类别分配框的颜色
            if box[-1] in self.classes_colors.keys():
                color = self.classes_colors[box[-1]]
            else:
                color = np.random.randint(0, 183, 3)
                self.classes_colors[box[-1]] = color

                # 绘制矩形框的函数 绘制的格式是RBG格式
            cv2.rectangle(img, (int(box[0]), int(box[1])),
                        (int(box[2]), int(box[3])), (int(color[0]),int(color[1]), int(color[2])), 2)
            if box[1] < 20:
                text_pos_y = box[1] + 30
            else:
                text_pos_y = box[1] - 10

            if box[-1] == 'basket' or ( box[-1] != 'basket' and 1.2 * (np.int64(box[2]) - np.int64(box[0])) > (np.int64(box[3]) - np.int64(box[1])) ):
                count += 1
                cv2.putText(img, box[-1],
                            (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
                # 将boundingBox数据存在boundingBoxes里
                boundingBox.num = np.int16(count)
                self.boundingBoxes.bounding_boxes.append(boundingBox)
            # 将边界框的信息发布出去
        self.position_pub.publish(self.boundingBoxes)
        cv2.line(img, (320, 1), (320, 450), (0, 0, 0), 1)
        cv2.imshow('YOLOv5', img)

def main():
    rospy.init_node('yolov5_ros', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":

    main()
