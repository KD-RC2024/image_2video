#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/BoundingBoxes.h"
#include "vector"
#include <opencv2/opencv.hpp>
#include "sensor_msgs/Image.h"
//找包 加 修改c++配置 来导入库文件
#include <Eigen/Eigen>
//包含这个头文件
#include <cv_bridge/cv_bridge.h>
//#include <image_transport/image_transport.h>
#include "algorithm"

#define LENG 30
//判断是否在其中
using namespace std;


std::vector<yolov5_ros_msgs::BoundingBox> ballR(LENG);
std::vector<yolov5_ros_msgs::BoundingBox> ballB(LENG);
//存储中间值 
std::vector<yolov5_ros_msgs::BoundingBox> res(10);
std::vector<yolov5_ros_msgs::BoundingBox> res2(10);
yolov5_ros_msgs::BoundingBox res_first;
yolov5_ros_msgs::BoundingBox res_ts;
int64_t x{},x1{}, y{},y4{};
int64_t miiin{};
int64_t dest{};
int64_t dep{};
int64_t Tep;
int64_t minn {};
int32_t recognize_sum {};
int16_t MODE = 0;
//弃用的参数
// int64_t r{};
// r = (res_first.xmin + res_first.xmax) / 2 - res_first.xmin;



void recognize_callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr & msg) {
    // 这里只有球的数据 
    // 完成数据的提取
    // 这里保留分开的红篮球数
    int16_t Red_num {};
    int16_t Blue_num {};
    recognize_sum = msg->bounding_boxes.size();
    for (int32_t i = 0; i < recognize_sum; ++i) {



        
        if (msg->bounding_boxes[i].Class == "blue") {
            ballB[Blue_num] = msg->bounding_boxes[i];
            Blue_num++;
        } 
        else if (msg->bounding_boxes[i].Class == "red") {
            ballR[Red_num] = msg->bounding_boxes[i];
            Red_num++;
        }
    }

    //第一次拉出一个初值 
    //大的抖动直接进入这个 重新计算
    if (Blue_num != 0 && MODE == 0) {
//对初始的res的提取
    minn = sqrt(((ballB[0].xmin + ballB[0].xmax) / 2 - 320) * ((ballB[0].xmin + ballB[0].xmax) / 2 - 320) 
                    + ((ballB[0].ymin + ballB[0].ymax) / 2 - 480) * ((ballB[0].ymin + ballB[0].ymax) / 2 - 480));

        for (int i = 0; i < Blue_num; ++i) {
            Tep = sqrt(((ballB[i].xmin + ballB[i].xmax) / 2 - 320) * ((ballB[i].xmin + ballB[i].xmax) / 2 - 320) 
                        + ((ballB[i].ymin + ballB[i].ymax) / 2 - 480) * ((ballB[i].ymin + ballB[i].ymax) / 2 - 480));
            if (minn >= Tep) {
                res_first = ballB[i];
                minn = Tep;
            }
            MODE = 1;
            res_ts = res_first;
        }
    }
    x1 = (res_first.xmin + res_first.xmax) / 2;
    y4 = (res_first.ymin + res_first.ymax) / 2;
    x = res_first.xmin;
    y = res_first.ymin;

    dest = sqrt(((res_first.xmin + res_first.xmax) / 2 - 320) * ((res_first.xmin + res_first.xmax) / 2 - 320) 
                    + ((res_first.ymin + res_first.ymax) / 2 - 480) * ((res_first.ymin + res_first.ymax) / 2 - 480));

    //这里还要考虑如果没有这个颜色的球
    if (MODE == 3 && Blue_num != 0) {
        //遍历球找出数据
        for (int i = 0; i < Blue_num; ++i) {
        //每一的距离
        Tep = sqrt(((ballB[i].xmin + ballB[i].xmax) / 2 - 320) * ((ballB[i].xmin + ballB[i].xmax) / 2 - 320) 
                    + ((ballB[i].ymin + ballB[i].ymax) / 2 - 480) * ((ballB[i].ymin + ballB[i].ymax) / 2 - 480));
            //如果说没有呢？？？
            //是不是可以goto 要保证有数据啊
            if (dest + 30 > Tep && Tep < dest - 30) {
                res.push_back(ballB[i]);
                
            }
        }
        //空的情况
        //很重要
        if (res.empty() == 1) {
            res.push_back(ballB[0]);
        }

        // 孤执一注

        for (int i = 0; i < res.size(); ++i) {
            if (res[i].xmin < 1.2 * x && res[i].xmin > 0.7 * x && res[i].ymin > 0.7 * y && res[i].ymin < 1.2 * y) {
                ROS_INFO("sssssss");
                res2.push_back(res[i]);
            }
        }

        if (res2.size() != 0) {
            res_ts = res2[0];
            miiin = sqrt((x1 - (res2[0].xmin + res[0].xmax) / 2) * (x1 - (res2[0].xmin + res2[0].xmax) / 2) 
                    + (y4 - (res2[0].ymin + res[0].ymax) / 2) * (y4 - (res2[0].ymin + res2[0].ymax) / 2));            
            for (int i =1; i < res2.size(); ++i) {
            dep = sqrt((x1 - (res2[i].xmin + res2[i].xmax) / 2) * (x1 - (res2[i].xmin + res2[i].xmax) / 2) 
                     + (y4 - (res2[i].ymin + res2[i].ymax) / 2) * (y4 - (res2[i].ymin + res2[i].ymax) / 2));
            if (miiin > dep) {
                miiin = dep;
                res_ts = res[i];             
            }
        } 
        ROS_INFO("%ld", res_ts.xmax);
        //记得更新res_first
        res_first = res_ts;
        MODE = 2;
        res.clear();
        res2.clear();                  
        }
        else {
            ROS_INFO("aaaaaaasss");
            res.clear();
            MODE = 0;
        }


        
    }
    if (Blue_num == 0)
        MODE = 0;

    }
    //距离比较
//     minn = ((ballR[0].xmin + ballR[0].xmax) / 2 - 320) * ((ballR[0].xmin + ballR[0].xmax) / 2 - 320) + ((ballR[0].ymin + ballR[0].ymax) / 2 - 480) * ((ballR[0].ymin + ballR[0].ymax) / 2 - 480);
//     for (int i = 0; i < Red_num; ++i) {
//         Tep = ((ballR[i].xmin + ballR[i].xmax) / 2 - 320) * ((ballR[i].xmin + ballR[i].xmax) / 2 - 320) + ((ballR[i].ymin + ballR[i].ymax) / 2 - 480) * ((ballR[i].ymin + ballR[i].ymax) / 2 - 480);
//         if (minn >= Tep) {
//             res = ballR[i];
//             minn = Tep;
//         }
//          MODE = 1;
//     }
    // 以下是红色的比较
    //320 480 比较


//以上只要把res_ts处理好就行
//只做了一件事情将目标的球框出来
void CV_DO (const sensor_msgs::Image::ConstPtr &igm) {

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(igm, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //cv::Mat image_self;
    //cv::cvtColor(cv_ptr->image, image_self, cv::COLOR_RGB2BGR);
    // //cv::rectangle(image_self, );
    // //point 是什么类型的
    cv::Scalar color;
    color[0] = 0; color[1] = 255; color[2] = 0;
    int32_t text_pos_y;
    
    if (MODE == 2 || MODE == 1) {
        cv::rectangle(cv_ptr->image, cvPoint(static_cast<int>(res_ts.xmin), static_cast<int>(res_ts.ymin)), cvPoint(static_cast<int>(res_ts.xmax), static_cast<int>(res_ts.ymax)),color, 2);
        MODE = 3;
        if (res_ts.ymin < 20)  
            text_pos_y = res_ts.ymin + 30;
        else
            text_pos_y = res_ts.ymin - 10;
    //绘制文本
        cv::putText(cv_ptr->image, "this", cvPoint(res_ts.xmin, text_pos_y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv::LINE_AA);
    }
    //cv::imshow("my_pictures", image_self);
    cv::imshow("my_pictures", cv_ptr->image);
    cv::waitKey(1);
}
int main(int argc , char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc , argv , "second_cam");
    ros::NodeHandle sh;
    ros::Subscriber Recognize = sh.subscribe<yolov5_ros_msgs::BoundingBoxes>("pub_topicK", 1, recognize_callback);
    ros::Subscriber CV_TO_DO = sh.subscribe<sensor_msgs::Image>("my_image", 1, CV_DO); 
    ros::spin();
    return 0;
}

/*
          备选方案
    通过类来封装实现函数的跳转
    订阅的是图像信息
    仿造的写
*/