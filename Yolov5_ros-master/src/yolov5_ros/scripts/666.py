#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import torch
import numpy as np
import os
# 初始化 权重模型初始化 保存的图片路径 保存的标签路径
    # 对图像进行处理 并通过模型进行数值预测
    # 这里的idx是序号
    # 绘制txt文件
if __name__ == "__main__":
    # 图片的路径
    image_dir = "/home/yami/2024_image2/5.22_2_images"
    # 存放txt的路径
    label_dir = "/home/yami/2024_image2/5.22_2_labes/"
    yolov5_path = "/home/yami/Yolov5_ros-master/src/yolov5_ros/yolov5"
    weight_path = "/home/yami/Yolov5_ros-master/src/yolov5_ros/weights/weightR/best.pt"
    conf = 0.65
    iou = 0.7
    model = torch.hub.load(yolov5_path, 'custom', path = weight_path, source = 'local')
    model.cuda()
    model.conf = conf
    model.iou = iou

    list_image = os.listdir(image_dir)

    for idx in range(0, 937):
        join_path = image_dir + "/" + str(idx) + ".jpg"
        color_image = cv2.imread(join_path)
        color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        results = model(color_image)
        boxs = results.pandas().xyxy[0].values
        newfile = label_dir +str(idx) + ".txt"
        new_file = open(newfile,"w")
        for box in boxs: 
        # boundingBox = BoundingBox()
        # boundingBox.probability =np.float64(box[4])
        # boundingBox.xmin = np.int64(box[0])
        # boundingBox.ymin = np.int64(box[1])
        # boundingBox.xmax = np.int64(box[2])
        # boundingBox.ymax = np.int64(box[3])
        # boundingBox.Class = box[-1]
        # out_file.write("{:} {:.6f} {:.6f} {:.6f} {:.6f} ".format(cls_id, *box) + '\n')
            if box[-1] == 'basket':
                xmax = np.int64(box[0] + box[2]) / 2560   # 640 * 2  2560
                ymax = np.int64(box[3] + box[1]) / 1440  # 480 * 2  1440
                xmin = np.int64(box[2] - box[0]) / 1280 # 640      1280
                ymin = np.int64(box[3] - box[1]) /  720 # 480      720
                # print(xmax)
                # print(ymax)
                # print(xmin)
                # print(ymin)
                tuple1 = (xmax, ymax, xmin, ymin)
                new_file.write("{:} {:.6f} {:.6f} {:.6f} {:.6f} ".format(str(0), *tuple1) + '\n')
            elif box[-1] == 'red':
                xmax = np.int64(box[0] + box[2]) / 2560   # 640 * 2  2560
                ymax = np.int64(box[3] + box[1]) / 1440  # 480 * 2  1440
                xmin = np.int64(box[2] - box[0]) / 1280 # 640      1280
                ymin = np.int64(box[3] - box[1]) /  720 # 480      720
                tuple1 = (xmax, ymax, xmin, ymin)
                new_file.write("{:} {:.6f} {:.6f} {:.6f} {:.6f} ".format(str(1), *tuple1) + '\n')
            elif box[-1] == 'blue':
                xmax = np.int64(box[0] + box[2]) / 2560   # 640 * 2  2560
                ymax = np.int64(box[3] + box[1]) / 1440  # 480 * 2  1440
                xmin = np.int64(box[2] - box[0]) / 1280 # 640      1280
                ymin = np.int64(box[3] - box[1]) /  720 # 480      720
                tuple1 = (xmax, ymax, xmin, ymin)
                new_file.write("{:} {:.6f} {:.6f} {:.6f} {:.6f} ".format(str(2), *tuple1) + '\n')
            else:
                xmax = np.int64(box[0] + box[2]) / 2560   # 640 * 2  2560
                ymax = np.int64(box[3] + box[1]) / 1440  # 480 * 2  1440
                xmin = np.int64(box[2] - box[0]) / 1280 # 640      1280
                ymin = np.int64(box[3] - box[1]) /  720 # 480      720
                tuple1 = (xmax, ymax, xmin, ymin)
                new_file.write("{:} {:.6f} {:.6f} {:.6f} {:.6f} ".format(str(3), *tuple1) + '\n')
        new_file.close() 
        print("完成" + str(idx))
