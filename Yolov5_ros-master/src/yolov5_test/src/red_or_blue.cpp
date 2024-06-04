/*
    1. 代码增加功能的实现
        A. 博弈代码(不影响已有代码)
        B. 红方 蓝方受上位板控制
        C. 数据的发送 
        D. 解决4的球的BUG
*/

/*
    A. 叠加红蓝方的代码完成
*/
/*
    博弈代码增加
    无球可放的时候进入博弈状态
    当在判断出此情况时进入通信机制的方案
    1. 我只需要把车导向3号框 并告诉他不要放球
*/

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <algorithm>
#include "yolov5_ros_msgs/BoundingBoxes.h"
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/port_serial.h"
#include "yolov5_ros_msgs/X_Y_ARG.h"
#include "yolov5_ros_msgs/M.h"
#include "std_msgs/Int32.h"
#include <unordered_set>
#include "unordered_map"

// 1 是 红色
// 2 是 蓝色
// 这里是链接的 使用的全局标志位
int Red = 0;
int Blue = 0;

//* A&B 阶段变量 
int32_t mode = -1;
//* 无序容器
std::unordered_set<int> My_set;
std::unordered_map<int, int> My_map;
//* 记录发送的数据
int Send_num {};
//* 全局复位
int fu_wei = -1;
// 考虑 如何发出去？？

//* 定义新的排序
int rank1[5] = {1, 2, 3, 4, 5};
int rank2[5] = {2, 3, 1, 4, 5};
int rank3[5] = {3, 2, 4, 1, 5};
int rank4[5] = {4, 3, 5, 2, 1};
int rank5[5] = {5, 4, 3, 2, 1};
//* 新的数据类型
// yolov5_ros_msgs::M O_o;
int32_t arr1[4];
int32_t arr2[4];
int32_t arr3[4];
int32_t arr4[4];
int32_t arr5[4];
// 从小到大
bool Bsk_Compare(const yolov5_ros_msgs::BoundingBox & a , const yolov5_ros_msgs::BoundingBox &b) {
    return a.xmin < b.xmin;
}
// 从大到小
bool Ball_Compare(const yolov5_ros_msgs::BoundingBox & a , const yolov5_ros_msgs::BoundingBox &b) {
    return a.ymin > b.ymin;
}
//* 决策函数 
int into_box(int arr[5][4], int rank[]) {
        for (int i = 0; i <= 4; ++i) {
            if (arr[i][0] == 2 && arr[i][2] == Blue)
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
            if (arr[rank[i] - 1][0] == 1 && arr[rank[i] - 1][1] == Red)
                return rank[i];
        }
        return 0;
}

void message_callback(const yolov5_ros_msgs::BoundingBoxes & msg) {
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
    // //* B状态临界值的比较
    int Temp[3]{};
    // //* 这里可能需要更改 
    std::vector<yolov5_ros_msgs::BoundingBox> basket;
    //* 每次传来数据的总数目
    if (!msg.bounding_boxes.empty()) {
    int16_t num_thing = msg.bounding_boxes[msg.bounding_boxes.size() - 1].num; 
    for (int i = 0; i < num_thing; ++i) {
        if (msg.bounding_boxes[i].Class == "basket") {
            Temp_basket++;
        }
    }
    if (Temp_basket == 5 && mode == 1) {
        for (int i = 0; i < num_thing; ++i) {
            if (msg.bounding_boxes[i].Class == "basket") {
                basket.push_back(msg.bounding_boxes[i]);
            }
        }
    std::sort(basket.begin(), basket.end(), Bsk_Compare);
    std::array<int, 5> sorted = {0, 1, 2, 3, 4};
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        return abs((basket[a].xmin + basket[a].xmax) / 2 - 320 ) < abs((basket[b].xmin + basket[b].xmax) / 2 - 320);});
    for (int q = 0; q < 5; ++q) {
        rank[q] = sorted[q] + 1;
    }   
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
            arr[Ha_num - 1][e] = Blue;    
            break;
        }
    }

    Ha_num *= 10;
    switch (Ha_num) {
        case 10 :
            Ha_num = Ha_num + into_box(arr, rank1);
            break;
        case 20 :
            Ha_num = Ha_num + into_box(arr, rank2);
            break;
        case 30 :
            Ha_num = Ha_num + into_box(arr, rank3);
            break;
        case 40 :
            Ha_num = Ha_num + into_box(arr, rank4);
            break;
        case 50 :
            Ha_num = Ha_num + into_box(arr, rank5);
            break;
        default:
        // 博弈期间
            break;
    }
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
// 初始化

// 现在应该明确 X_Y_ARG 是接受的数据 而 port_serial 是发送的数据

// 用x的值作为红方与蓝方
// 初始化


void TD_location(const yolov5_ros_msgs::X_Y_ARG & msg) {
    static ros::NodeHandle nh;
    static ros::Publisher message_pub = nh.advertise<yolov5_ros_msgs::port_serial>("basket_id", 10);
    static yolov5_ros_msgs::port_serial Bask_ID;
    // 红方
    if (msg.x == 1) {
        Red = 1;
        Blue = 2;
        ROS_INFO("My are red!!");
    }
    // 
    else if (msg.x == 2) {
        Red = 2;
        Blue = 1;
        ROS_INFO("My are blue!!");
    }
    else {
        ROS_INFO("Have a problen");
    }


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
    ros::Subscriber location = sh.subscribe("X_Y_ARG", 1, TD_location);
    ros::Subscriber test_location = sh.subscribe("Key_mode", 1, Key_location);
    //* 这里是否使用循环多线程
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    return 0;
}