/*
    红方代码
*/
// 另一种决策模式
// 大概有5个阶段
// 1. 跑3区全程
// 2. 炼数据集
// 3. 跑3区全程
// 4. 炼数据集
// 5. 


/*
    3 帧 加快判断 

    补充手段
    转舵机 


*/

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <algorithm>
#include "yolov5_ros_msgs/BoundingBoxes.h"
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/port_serial.h"
#include "yolov5_ros_msgs/X_Y_ARG.h"
#include "std_msgs/Int32.h"
#include <unordered_set>
#include "unordered_map"
#define blue 2
#define red 1

//*  全局变量  *//
//* A&B 阶段变量 
int32_t mode = -1;
//* 无序容器
std::unordered_set<int> My_set;
std::unordered_map<int, int> My_map;
//* 记录发送的数据
int Send_num {};
//* 全局复位
int fu_wei = -1;

// 从小到大
bool Bsk_Compare(const yolov5_ros_msgs::BoundingBox & a , const yolov5_ros_msgs::BoundingBox &b) {
    return a.xmin < b.xmin;
}
// 从大到小
bool Ball_Compare(const yolov5_ros_msgs::BoundingBox & a , const yolov5_ros_msgs::BoundingBox &b) {
    return a.ymin > b.ymin;
}
// //* 决策函数 
int into_box(int arr[5][4], int rank[]) {
        // 这里是判断及其特殊的情况 red是对方的球
        for (int i = 0; i <= 4; ++i) {
            if (arr[i][0] == 2 && arr[i][2] == blue)
                return i + 1;
        }
        // 这里是空框的情况
        for (int i = 0; i <= 4; ++i) {
            if (arr[rank[i] - 1][0] == 0)  //arr[i][0] == 2 || arr[i][0] == 0
                return rank[i];
        }
        // 这里是两个球的情况
        for (int i = 0; i <= 4; ++i) {
            if (arr[rank[i] - 1][0] == 2)
                return rank[i];
        }
        // 这里补充的情况
        for (int i = 0; i <= 4; ++i) {
            if (arr[rank[i] - 1][0] == 1 && arr[rank[i] - 1][1] == red)
                return rank[i];
        }
        return 9;
}


//* 全局变量
void message_callback(const yolov5_ros_msgs::BoundingBoxes & msg) {
    //*** 每次需要更新的数据 ***//
    //* 记录每次框的数量
    int Temp_basket = 0;
    // //* 用与记录球的容器
    std::vector<yolov5_ros_msgs::BoundingBox> ball;
    //* 与记录球容器一起使用的记录总数的变量
    int Sum_ball{};
    // //* 传入函数使用
    int rank[5]{};
    // //* arr[5][4]
    int arr[5][4] = {0};
    // //* 进行哈希表的数据
    int Ha_num{};
    // //* B使用的中心框变量
    yolov5_ros_msgs::BoundingBox Zhong_xing;
    // //* B状态下临界值的比较
    int Temp[3]{};
    // //* 这里可能需要更改 
    std::vector<yolov5_ros_msgs::BoundingBox> basket;
    //* 每次传来数据的总数目
    if (!msg.bounding_boxes.empty()) {
    //** 识别框的数目构建完成
        ROS_INFO("aaa");
        
        int16_t num_thing = msg.bounding_boxes[msg.bounding_boxes.size() - 1].num; 
        for (int i = 0; i < num_thing; ++i) {
            if (msg.bounding_boxes[i].Class == "basket") {
                Temp_basket++;
            }
        }
    //** 框的容器构建完成
    if (Temp_basket == 5 && mode == 1) {
        for (int i = 0; i < num_thing; ++i) {
            if (msg.bounding_boxes[i].Class == "basket") {
                basket.push_back(msg.bounding_boxes[i]);
            }
        }
    std::sort(basket.begin(), basket.end(), Bsk_Compare);
    //** rank数据构建完成
    std::array<int, 5> sorted = {0, 1, 2, 3, 4};
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        return abs((basket[a].xmin + basket[a].xmax) / 2 - 320 ) < abs((basket[b].xmin + basket[b].xmax) / 2 - 320);});
    for (int q = 0; q < 5; ++q) {
        rank[q] = sorted[q] + 1;
    }   

    //** arr[5][4]数据构建完成 
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < num_thing; ++j) {
                    if (msg.bounding_boxes[j].Class != "basket" && 
                        (msg.bounding_boxes[j].xmin + msg.bounding_boxes[j].xmax) / 2 > basket[i].xmin - 5 &&
                        (msg.bounding_boxes[j].xmin + msg.bounding_boxes[j].xmax) / 2 < basket[i].xmax + 5 && 
                        (msg.bounding_boxes[j].ymin + msg.bounding_boxes[j].ymax) / 2 > basket[i].ymin - 30 &&
                        (msg.bounding_boxes[j].ymin + msg.bounding_boxes[j].ymax) / 2 < basket[i].ymax + 5) {
                            ball.push_back(msg.bounding_boxes[j]);
                            Sum_ball++;
                }
        }
        std::sort(ball.begin(), ball.end(), Ball_Compare);
        arr[i][0] = Sum_ball;
        for (int m = 0; m < Sum_ball; ++m) {
            if (ball[m].Class == "red") {
                arr[i][m + 1] = 1;
            } else {
                arr[i][m + 1] = 2;
            } 
        }
        Sum_ball = 0;
        ball.clear();
    }
    Ha_num = into_box(arr, rank);
    for (int e = 1; e < 4; ++e) {
        if (arr[Ha_num - 1][e] == 0) {
            arr[Ha_num - 1][0]++;
            arr[Ha_num - 1][e] = blue;    
            break;
        }
    }
    Ha_num *= 10;
    // 这里传的rank需要重新评定

    // 这里传的rank需要重新评定
    Ha_num = Ha_num + into_box(arr, rank);
    if (My_set.find(Ha_num) == My_set.end()) {
        My_set.insert(Ha_num);
        My_map.emplace(Ha_num, 1);
    }
    else if (My_set.find(Ha_num) != My_set.end()) {
        My_map[Ha_num]++;
    }
    }
    basket.clear();
    }
} 
void TD_location(const yolov5_ros_msgs::X_Y_ARG & msg) {
    static ros::NodeHandle nh;
    static ros::Publisher message_pub = nh.advertise< yolov5_ros_msgs::port_serial>("basket_id", 10);
    static yolov5_ros_msgs::port_serial Bask_ID;
    if (msg.mode == 1) {
        // 进入运行模式
        if (fu_wei == 1) {
            Send_num = 0;
            My_map.clear();
            My_set.clear();
        }
        fu_wei = 0;
        mode = msg.mode;
    }
    if (msg.mode == 2) {
        // 进入发送模式
        if (fu_wei == 0) {
            int Temp{};
            for (int num : My_set) {
                if (My_map[num] >= Temp) {
                    Temp = My_map[num];
                    Send_num = num;
                }
            }
            fu_wei = 1;
        }
        mode = msg.mode;
        Bask_ID.id = Send_num / 10;
        ROS_INFO("%d", Send_num);
        message_pub.publish(Bask_ID);
    }
}
void Key_location(const std_msgs::Int32 & msg) {
    static ros::NodeHandle nh;
    static ros::Publisher message_pub = nh.advertise< yolov5_ros_msgs::port_serial>("basket_id", 10);
    static yolov5_ros_msgs::port_serial Bask_ID;
    if (msg.data == 1) {
        // 进入运行模式
        if (fu_wei == 1) {
            Send_num = 0;
            My_map.clear();
            My_set.clear();
        }
        fu_wei = 0;
        mode = msg.data;
        ROS_INFO("1");
    }
    if (msg.data == 2) {
        // 进入发送模式
        if (fu_wei == 0) {
            int Temp{};
            for (int num : My_set) {
                if (My_map[num] >= Temp) {
                    Temp = My_map[num];
                    Send_num = num;
                }
            }
            fu_wei = 1;
        }
        ROS_INFO("2");
        ROS_INFO("%d", Send_num);
        mode = msg.data;
        Bask_ID.id = Send_num / 10;
        message_pub.publish(Bask_ID);
    }
}
int main(int argc , char *argv[]) {
    setlocale(LC_ALL , " " );
    ros::init(argc , argv , "message");
    ros::NodeHandle sh;
    ros::Subscriber message_sub = sh.subscribe("/yolov5/BoundingBoxes" , 10 , message_callback);
    //ros::Subscriber location = sh.subscribe("X_Y_ARG", 1, TD_location);
    ros::Subscriber test_location = sh.subscribe("Key_mode", 1, Key_location);
    //* 这里是否使用循环多线程
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}