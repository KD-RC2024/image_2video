/*

    红方代码
    
*/
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <algorithm>
#include "yolov5_ros_msgs/BoundingBoxes.h"
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/port_serial.h"
#include "yolov5_ros_msgs/X_Y_ARG.h"
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
//* 记录平均次数
int Arv_basket;
//* 记录发送的数据
int Send_num {};
//* 在模式B下的更改发送变量
bool Send_or_Send = 1;
//* B状态下的值的比较
int Str[4] {};
//* B状态下不同的次数
int dif{};
//* msssss
int mmmm = 0;
//* 默认为不相同的值
bool AAAA__BBBB = 0;
//* 全局复位
int fu_wei = 0;
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
    /** 
     *  命令的发布方 
     *  
     *  效果是始终需要一个ID 且是不断靠近框的过程中 什么时候进行ID进行转换(考虑的）
     *  
     *  捕捉框的过程中 (繁琐)
     *  ////
     *  进入到A阶段 静止
     *  第一次的判断必须要出现5个框的时候 出现 1个选择 1个后补 加上根据中心线之间的距离进行判断 多次判断 发送那个次数出现最多的数据(可以用)
     *  ////
     *  发送数据 给上位板
     *  ////
     *  进入到B阶段 运动阶段 及其慎重的更改发送的ID 更像是一个挂起阶段 随着每帧数据的输入而发生更改
     *  什么情况下更改
     *  在运动情况下 增加平均值的数目 这里只需要判断中心线的框的情况 50个等等这里测试就好了
     *  ////
    **/
    //* 发送方实现
    static ros::NodeHandle nh;
    static ros::Publisher message_pub = nh.advertise< yolov5_ros_msgs::port_serial>("basket_id", 10);
    static yolov5_ros_msgs::port_serial Bask_ID;
   ///////////// 阶段A的实现
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
                        (msg.bounding_boxes[j].ymin + msg.bounding_boxes[j].ymax) / 2 < basket[i].ymax) {
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
    //////arr[5][4] 构建的没问题
    // ROS_INFO("strat");
    // for (int i = 0; i < 5; ++i) {
    //     for (int j = 0; j < 4; ++j) {
    //         ROS_INFO("%d", arr[i][j]);
    //     }
    //     ROS_INFO("\n");
    // }
    // ROS_INFO("end");

    //** 发送数据数据Send_num构建完成
    Ha_num = into_box(arr, rank);
    for (int e = 1; e < 4; ++e) {
        if (arr[Ha_num - 1][e] == 0) {
            arr[Ha_num - 1][0]++;
            arr[Ha_num - 1][e] = blue;    
            break;
        }
    }
    // ROS_INFO("strat");
    // for (int i = 0; i < 5; ++i) {
    //     for (int j = 0; j < 4; ++j) {
    //         ROS_INFO("%d", arr[i][j]);
    //     }
    //     ROS_INFO("\n");
    // }
    // ROS_INFO("end");
    Ha_num *= 10;
    Ha_num = Ha_num + into_box(arr, rank);
    if (My_set.find(Ha_num) == My_set.end()) {
        My_set.insert(Ha_num);
        My_map.emplace(Ha_num, 1);
        Arv_basket++;
    }
    else if (My_set.find(Ha_num) != My_set.end()) {
        My_map[Ha_num]++;
        Arv_basket++;
    }
    //* 这里是精度的把控
    if (Arv_basket >= 25) {
        int Temp{};
            for (int num : My_set) {
                if (My_map[num] >= Temp) {
                    Temp = My_map[num];
                    Send_num = num;
                }
            }
        // ROS_INFO("%d", Send_num);
        // 切换模式
        mmmm = 1;
    }
    }
///////////// 阶段B的实现
    if (mmmm == 1 && mode == 1) {
        //* 注意这里的Send_or_Send
        if (Send_or_Send == 1) {
                Bask_ID.id = Send_num / 10;
                // 这次的数据可能有问题
                // 需要解决 进来的一瞬间 出现问题 真的不可靠
                if (AAAA__BBBB == 0) {
                    if (arr[Bask_ID.id - 1][0] == 3) {
                        Str[0] = arr[Bask_ID.id - 1][0] - 1;
                        Str[1] = arr[Bask_ID.id - 1][1]; 
                        Str[2] = arr[Bask_ID.id - 1][2];
                        Str[3] = 0;
                        AAAA__BBBB = 1;
                    } 
                    else if (arr[Bask_ID.id - 1][0] < 3) {
                        Str[0] = arr[Bask_ID.id - 1][0];
                        Str[1] = arr[Bask_ID.id - 1][1]; 
                        Str[2] = arr[Bask_ID.id - 1][2];
                        Str[3] = 0;
                        AAAA__BBBB = 1;
                    }
                }
                message_pub.publish(Bask_ID);
                ROS_INFO("%d", Bask_ID.id);
            }
        else {
            Bask_ID.id = Send_num / 10;
            message_pub.publish(Bask_ID);
            ROS_INFO("%d", Bask_ID.id);
        }
            if (Send_or_Send) {
                // ROS_INFO("strat");
                //     for (int j = 0; j < 3; ++j) {
                //         ROS_INFO("%d", Str[j]);
                //     }
                //     ROS_INFO("\n");
                // ROS_INFO("end");
            for (int i = 0; i < num_thing; ++i) {
                if (msg.bounding_boxes[i].Class == "basket") {
                    basket.push_back(msg.bounding_boxes[i]);
                }   
            }

            if (basket.size() != 0) {
                //** 给定中心框构建完成
                std::sort(basket.begin(), basket.end(), Bsk_Compare);
                Zhong_xing = basket[0];
                for (int q = 0; q < Temp_basket; ++q) {
                    if ( abs((basket[q].xmin + basket[q].xmax) / 2 - 320) < abs((Zhong_xing.xmin + Zhong_xing.xmax) / 2 - 320)) {
                        Zhong_xing = basket[q];
                    }
                } 
                // ROS_INFO("%ld", abs((Zhong_xing.xmin + Zhong_xing.xmax)/2-320));
                //** 中心框的球构建完成
                for (int j = 0; j < num_thing; ++j) {
                    if (msg.bounding_boxes[j].Class != "basket" && 
                        (msg.bounding_boxes[j].xmin + msg.bounding_boxes[j].xmax) / 2 > Zhong_xing.xmin - 5 &&
                        (msg.bounding_boxes[j].xmin + msg.bounding_boxes[j].xmax) / 2 < Zhong_xing.xmax + 5 && 
                        (msg.bounding_boxes[j].ymin + msg.bounding_boxes[j].ymax) / 2 > Zhong_xing.ymin - 30 &&
                        (msg.bounding_boxes[j].ymin + msg.bounding_boxes[j].ymax) / 2 < Zhong_xing.ymax) {
                            ball.push_back(msg.bounding_boxes[j]);
                            Sum_ball++;
                    }
                }
                std::sort(ball.begin(), ball.end(), Ball_Compare);

                //** 累加不同的情况
                if (Sum_ball >= Str[0]) {
                    //** 如果是0个 不会进循环 
                    for (int i = 0; i < Sum_ball; ++i) {
                        int Temp_1 = 0;
                        // 这里近距离的数据非常不稳
                        if (ball[i].Class == "red") {
                            Temp_1 = 1;
                        } else {
                            Temp_1 = 2;
                        }
                        if (Temp_1 != Str[i + 1] ) {
                            dif++;
                            break;
                        }
                    }
                }
            }
            //** 这里怎么调整
            if (dif >= 200) {
                Send_or_Send = 0;
            }
                Sum_ball = 0;
                ball.clear();
            }
    }
    basket.clear();
    }
} 
void TD_location(const yolov5_ros_msgs::X_Y_ARG & msg) {
    if (msg.mode == 1) {
        // 进入发送模式
        fu_wei = 0;
        mode = msg.mode;
    }
    if (msg.mode == 2) {
        // 进入不发送模式
        // 复位模式
        if (fu_wei == 0) {
            Send_or_Send = 1;
            mmmm = 0;
            AAAA__BBBB = 0;
            Arv_basket = 0;
            Send_num = 0;
            Send_or_Send = 1;
            Str[0] = 0;
            Str[1] = 0;
            Str[2] = 0;
            Str[3] = 0;
            My_set.clear();
            My_map.clear();
            dif = 0;
            fu_wei = 1;
        }
        mode = msg.mode;
    }
}
int main(int argc , char *argv[]) {
    setlocale(LC_ALL , " " );
    ros::init(argc , argv , "message");
    ros::NodeHandle sh;
    ros::Subscriber message_sub = sh.subscribe("/yolov5/BoundingBoxes" , 10 , message_callback);
    ros::Subscriber location = sh.subscribe("X_Y_ARG", 1, TD_location);
    //* 这里是否使用循环多线程
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}