#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes
# xmin    ymin    xmax   ymax  confidence  class   name

class Yolo_Dect:
    # 初始化 权重模型初始化
    # 发布者消息初始化
    def __init__(self):

        # 权重模型导入 及初始化
        yolov5_path = rospy.get_param('/yolov5_path', '')
        weight_pathK = rospy.get_param('~weight_path', '')
        #weight_pathR = rospy.get_param('~weight_pathR', '')
        #weight_pathB = rospy.get_param('~weight_pathB', '')
        conf = rospy.get_param('~conf', '0.5')
        self.modelK = torch.hub.load(yolov5_path, 'custom', path=weight_pathK, source='local')
        #self.modelR = torch.hub.load(yolov5_path, 'custom', path=weight_pathR, source='local')
        #self.modelB = torch.hub.load(yolov5_path, 'custom', path=weight_pathB, source='local')

        if (rospy.get_param('/use_cpu', 'false')):
            self.modelK.cpu()
            #self.modelR.cpu()
            #self.modelB.cpu()
        else:
            self.modelK.cuda()
            #self.modelR.cuda()
            #self.modelB.cuda()

        # 给模型传入阈值参数
        self.modelK.conf = conf
        #self.modelR.conf = conf
        #self.modelB.conf = conf

        # 图像数据的处理接收与发布
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        pub_topicK = rospy.get_param('~pub_topicK', 'pub_topicK')
        #pub_topicR = rospy.get_param('~pub_topicR', 'pub_topicR')
        #pub_topicB = rospy.get_param('~pub_topicB', 'pub_topicB')
        #pub_topicRB = rospy.get_param('~pub_topicRB', 'pub_topicRB')
        # 发布图像的时候用到了
        # self.camera_frame = rospy.get_param('~camera_frame', '')
        # 新的想法
        # 模型导入 3个模型 分层匹配 


        
        
        # sensor_msgs.msg 里面的ros 图像初始化
        # 初始化两个Image
        self.color_image = Image()
        # 这个getImageStatus的标志位
        self.getImageStatus = False

        # 这个是矩形的颜色设定
        self.classes_colors = {}

        # image subscribe 通过回调函数处理
        # 用于处理Image的图像类型 去订阅  50MB的订阅数据量 
        # 接收图像的 接受ros usb的图像

        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800)  
        # 发布者实现 publishers  queue_size给1 类似实现代码上的实时
        # 实现位置的发布 发布最新的消息
        self.image1 = rospy.Publisher("my_image", Image, queue_size=1)

        self.position_pubK = rospy.Publisher(pub_topicK, BoundingBoxes, queue_size=1)
        #self.position_pubR = rospy.Publisher(pub_topicR, BoundingBoxes, queue_size=1)
        #self.position_pubB = rospy.Publisher(pub_topicB, BoundingBoxes, queue_size=1)
        #self.position_pubRB = rospy.Publisher(pub_topicRB, BoundingBoxes, queue_size=1)
        # Image 是消息类型   这里发布的是处理后的图片 
        # self.image_pub = rospy.Publisher('/yolov5/detection_image',  Image, queue_size=1)
        # if no image messages
        while (not self.getImageStatus):
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)




    # 对图像进行处理 并通过模型进行数值预测
    def image_callback(self, image):
        # 自定义消息类型 在python中需要初始化
        self.boundingBoxes = BoundingBoxes()
        self.boundingBoxes.header = image.header
        # rospy.loginfo(self.boundingBoxes.header)
        self.boundingBoxes.image_header = image.header
        self.getImageStatus = True
        self.image1.publish(image)

        # 重塑ROS类型 这里把color_image转化成RGB的形式  从ros图像到cv图像
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        # 用cv 进行 图像颜色处理       
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)



        # 开始执行 模型是需要cv形式的图像的
        resultsK = self.modelK(self.color_image)
        #resultsR = self.modelR(self.color_image)
        #resultsB = self.modelB(self.color_image)
        # 增加的数据
        
        # 使用 YOLOv5 模型检测目标后得到的边界框（bounding boxes）的表示形式之一
        boxsK = resultsK.pandas().xyxy[0].values
        #boxsR = resultsR.pandas().xyxy[0].values
        #boxsB = resultsB.pandas().xyxy[0].values
        # color_image色彩转换后的图像 输入处理层
        self.dectshow(self.color_image, boxsK)
        #self.dectshow(self.color_image, boxsR)
        #self.dectshow(self.color_image, boxsB)
        # cv2中的必须品
        cv2.waitKey(3)
        #self.imshoww(self.color_image)

    # 传入RBG的图像 目标的一些数值
    def dectshow(self, img, boxs):
        # img = org_img.copy()

        count = 0
        for i in boxs:
            # 一张图片中有多少个检测物体
            count += 1
        for box in boxs:
            # 在已有数据的情况下对数据进行np处理     
            boundingBox = BoundingBox()
            # 最终的时候不增加
            # boundingBox.probability =np.float64(box[4])
            # 最终的时候不增加
            # 需要的
            boundingBox.xmin = np.int64(box[0])
            boundingBox.ymin = np.int64(box[1])
            boundingBox.xmax = np.int64(box[2])
            boundingBox.ymax = np.int64(box[3])
            boundingBox.Class = box[-1]

            # 如果说想法成立 则是必须得值
            boundingBox.num = np.int16(count)
            # 为每种类别分配框的颜色
            if box[-1] in self.classes_colors.keys():
                color = self.classes_colors[box[-1]]
            else:
                color = np.random.randint(0, 183, 3)
                self.classes_colors[box[-1]] = color

            # 绘制矩形框的函数 绘制的格式是RBG格式
            cv2.rectangle(img, (int(box[0]), int(box[1])),
                          (int(box[2]), int(box[3])), (int(color[0]),int(color[1]), int(color[2])), 2)
            #rospy.loginfo("%d", int(box[0]))
            if box[1] < 20:
                text_pos_y = box[1] + 30
            else:
                text_pos_y = box[1] - 10

            # 绘制文本
            cv2.putText(img, box[-1],
                        (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)
            # 将boundingBox数据存在boundingBoxes里
            self.boundingBoxes.bounding_boxes.append(boundingBox)
            # 将边界框的信息发布出去
        self.position_pubK.publish(self.boundingBoxes)
        cv2.imshow('YOLOv5', img)
        # 发布图像的函数调用
    #    self.publish_image(img, height, width)
    #def imshoww(self,img):
        #cv2.imshow('YOLOv5', img)

    # def publish_image(self, imgdata, height, width):
    #     # 初始化ROS _ Image消息类型
        

    #     image_temp = Image()


    #     header = Header(stamp=rospy.Time.now())
    #     header.frame_id = self.camera_frame
    #     image_temp.height = height
    #     image_temp.width = width
        # encoding 编码实现
    #     image_temp.encoding = 'bgr8'
        

    #     image_temp.data = np.array(imgdata).tobytes()
        

    #     image_temp.header = header
    #     image_temp.step = width * 3
    #     # 将处理后的图像发出
    #     self.image_pub.publish(image_temp)





def main():
    rospy.init_node('yolov5_ros', anonymous=True)
    yolo_dect = Yolo_Dect()
    rospy.spin()


if __name__ == "__main__":

    main()
