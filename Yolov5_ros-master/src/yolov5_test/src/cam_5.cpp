/*    */
    // 蓝方代码
/*    */


//* 头文件区域*******************************
#include "ros/ros.h"
#include <std_msgs/String.h>
#include <algorithm>
#include "vector"
#include "yolov5_ros_msgs/BoundingBoxes.h"
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/port_serial.h"
#include "yolov5_ros_msgs/X_Y_ARG.h"
#include "cmath"
#include <unordered_set>
#include "unordered_map"
#include <tuple>
//* 头文件区域*******************************


//* 宏定义区域*******************************
#define red 1
#define blue 2
#define PI 3.14159265
//* 宏定义区域*******************************


//* 全局变量区域*****************************
//* 核心数据层
int arr[5][4] = {0};
//* 坐标与角度的需要时更新的信息
double _X = 2000;
double _Y = 1125;
double _aarg = 2.6;
//* 这的二维数组是用来存储框的世界坐标的
int basket_world_location[5][2] = {375, 625, 
                                   1125, 1375,
                                   1875, 2125,
                                   2625, 2875,
                                   3375, 3825};
//* 全局排序函数实时更新使用
int rank[5] = {3, 2, 4, 1, 5};
//* 用与框里球状态的叠加
int Dp_Dif[5] = {0};
//* 大笼统将次计数
int Big_Num = 3;
//* 对面机器人持球导致失误判断的变量
int VS_Robot[5] = {0};
//* 实时更新的坐标与角度
int16_t New_X = 0;
int16_t New_Y = 0;
int16_t New_Arg = 0;
//* 获取更新数据标志位
bool New_Get = 0;
//* 接受决策的ID号
int64_t ID_Now = 0;
//* 是否中途更换ID
bool Change_or_Not_Change = 0;
//* 投完球之后的标志位
bool New_Change = 0;
//* 确实投完了 的 标志位
bool IS_IS_OK = 0;


//* 无序容器
std::unordered_set<int> My_set;
std::unordered_map<int, int> My_map;
//* 记录平均次数
int Arv_basket;
//* 记录发送的数据
int Send_num {};
//* B状态下不同的次数
int dif{};

//* 全局变量区域*******************************


//* 外部接口函数区域****************************
//* 框的从小到大比较vector接口函数
bool Bsk_Compare(const yolov5_ros_msgs::BoundingBox & a , const yolov5_ros_msgs::BoundingBox & b) {
    return a.xmin < b.xmin;
}
//* 球的从下往上比较vector接口函数
bool Ball_Compare(const yolov5_ros_msgs::BoundingBox & a , const yolov5_ros_msgs::BoundingBox & b) {
    return a.ymin > b.ymin;
}

//* 决策函数 这里的调整用作
int into_box(int arr[5][4], int rank[]) {
        // 这里是判断及其特殊的情况 red是对方的球
        for (int i = 0; i <= 4; ++i) {
            if (arr[i][0] == 2 && arr[i][2] == red)
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
            if (arr[rank[i] - 1][0] == 1 && arr[rank[i] - 1][1] == blue)
                return rank[i];
        }
        return 9;
}
//* 左右角度转化函数
double my_R_arg(double arg) {
    double Temp = 23 + arg;
    if (Temp < 0)
        Temp = 360 + Temp;
    return Temp;
}
double my_L_arg(double arg) {
    double Temp = 337 + arg;
    if (Temp > 360)
        Temp = Temp - 360;
    return Temp;
}
//* 输出框的vector序列容器 这里需求返回一个元组
std::tuple<int, int, std::vector<int>> From_X_Y_arg(double _X, double _Y, double _aarg) {
    std::vector<int> arr(1, 5);
    double _arg_L, _arg_R;
    double Lift, Right;
    double Sq_1 = 6000 - _Y;
    if ( !(_aarg > 90 || _aarg < -90)) {
        // 这个角度完全可以给零的
        _arg_R = my_R_arg(_aarg);
        _arg_L = my_L_arg(_aarg);
        // 都在左边
        if (_arg_R > 150 && _arg_L > 150) {
            _arg_L = 360 - _arg_L;
            _arg_R = 360 - _arg_R;
            Lift = _X + tan(_arg_L * PI / 180.0) * Sq_1;
            Right = _X + tan(_arg_R * PI / 180.0) * Sq_1;
        }
        // 都在右边
        else if (_arg_R < 150 && _arg_L < 150 ) {
            Lift = _X - tan(_arg_L * PI / 180.0) * Sq_1;
            Right = _X - tan(_arg_R * PI / 180.0) * Sq_1;
        }
        // 一左一右
        else {
            _arg_L = 360 - _arg_L;
            Lift = _X + tan(_arg_L * PI / 180.0) * Sq_1;
            Right = _X - tan(_arg_R * PI / 180.0) * Sq_1;
        }
            if (Lift <= 3500) {
                arr[1] = 4;
                if (Lift <= 2750) {
                    arr[1] = 3;
                    if (Lift <= 2000) {
                        arr[1] = 2;
                        if (Lift <= 1250) {
                            arr[1] = 1;
                            if (Lift <= 500)
                                arr[1] = 0;
                        }
                    }
                }
            }
            if (Right <= 3500) {
                arr[0] = 5;
                if (Right <= 2750) {
                    arr[0] = 4;
                    if (Right <= 2000) {
                        arr[0] = 3;
                        if (Right <= 1250) {
                            arr[0] = 2;
                            if (Right <= 500)
                                arr[0] = 1;
                        }
                    }
                }
            }
            if (Right >= 3500) {
                arr[0] = 1;
                arr[1] = 0;
            }
    }
    std::tuple<int, int, std::vector<int>> My_tuple{Lift, Right, arr};
    return My_tuple;
} 
//* 外部接口函数区域*****************************

//** 这个线程就是在处理 这个arr[][] 数据的容器的构建 **//
void message_callback(const yolov5_ros_msgs::BoundingBoxes & msg) {
    /**
     * 
     *    同时我应该遵循 一但发送数据非必要不变原则 
     *    5.10 我需要为每一个arr[]建立一个纠错容器库 用来把握动态变化 需要时甩入决策函数里面
     *    5.10 根据比例范围预测框的位置
     *    5.11 决策之前需要一个排序
     * 
     * 
     *  这里建立纠错容器需要注意:
     *  1. 如果一定时间没有某一框的数据
     *  2. 如果一个框满了
     *  3. 怎么构建更新机制
     *  4. 在每一次得到数据的时候进入 纠错容器
     *  5. 明确会改变的情况只有两种 1. 是我方放了 2. 是对方放了
     *  排序之后的这个Temp_arr[]的控制 用于决策数据的更改问题
     * 
     *  防寄机制 :
     *  1. 防止突发的意外情况
     *  2. 防止坐标疯掉
     *  3. ... 
     *  全局rank实时更新 需要时调用 与之前一样
     *  
    **/
    //* 进行哈希表的数据
    int Ha_num{};
    //* B使用的中心框变量
    // yolov5_ros_msgs::BoundingBox Zhong_xing;
    //* B状态下临界值的比较
    int Temp[3]{};


    //* 记录每次框的数量
    int Temp_basket = 0;
    //* 与记录球容器一起使用的 记录每个框球的总数的变量
    int Sum_ball{};
    //* 用与记录球的容器
    std::vector<yolov5_ros_msgs::BoundingBox> ball;
    //* 这里需要的是介值数组 记录数量的值初始化为-1
    int Temp_arr[5][4] = {0};
    for (int i = 0; i < 5; ++i)
        Temp_arr[i][0] = -1;
    //* 这里可能需要更改 
    std::vector<yolov5_ros_msgs::BoundingBox> basket;
    //* 定义一系列接受元组的变量
    int Life_From_tuple, Right_From_tuple;
    std::vector<int> basket_rank;
    //* 用来存储预测框位置的数组变量
    int Guest_arr[5][2] = {0};
    //* 用于纠错容器的全局增球计数
    int Add_Ball_Num = 0;
    //* 处于回调函数内部的变量****************************


    if (!msg.bounding_boxes.empty()) {
        //* Temp_basket是框的数目
        //* num_thing是所有被识别出来的数目
        int16_t num_thing = msg.bounding_boxes[msg.bounding_boxes.size() - 1].num; 
        for (int i = 0; i < num_thing; ++i) {
            if (msg.bounding_boxes[i].Class == "basket")
                Temp_basket++;
        }

        //* 从另一个回调函数里获取位置信息并做处理
        New_Get = 1;
        ros::Duration(0.01).sleep();
        _X = New_X;
        _Y = New_Y;
        _aarg = New_Arg;
        std::tie(Life_From_tuple, Right_From_tuple, basket_rank) = From_X_Y_arg(_X, _Y, _aarg);
        New_Get = 0;

        //* 以下进入Temp_arr 数组的实现 
        if (Temp_basket == basket_rank[1] - basket_rank[0] + 1) {
            if (Temp_basket == 0) {
                // 直接走人 重新识别
                basket_rank.clear();
                ROS_INFO("视野中没有框 只有漂亮的 娇娇公主 ");
                return;
            }
            // 切记vector容器的释放
            // 测试的时候需要手动命令行修改参数这个功能
            //** 这里完成框的排序 与 排序函数
            for (int i = 0; i < num_thing; ++i) {
                if (msg.bounding_boxes[i].Class == "basket")
                    basket.push_back(msg.bounding_boxes[i]);
            }
            std::sort(basket.begin(), basket.end(), Bsk_Compare);
            std::array<int, 5> sorted = {0, 1, 2, 3, 4};
            std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
                return abs((basket_world_location[a][0] + basket_world_location[a][1]) / 2 - (Life_From_tuple + Right_From_tuple) / 2 ) < abs((basket_world_location[b][0] + basket_world_location[b][1]) / 2 - (Life_From_tuple + Right_From_tuple) / 2);});
            for (int q = 0; q < 5; ++q) 
                rank[q] = sorted[q] + 1;

            //** 完成Temp_arr数组
            for (int i = basket_rank[0] - 1, k = 0; i < basket_rank[1]; ++i) {
                for (int j = 0; j < num_thing; ++j) {
                    // 这里最好是增加比例识别
                        if (msg.bounding_boxes[j].Class != "basket" && 
                            (msg.bounding_boxes[j].xmin + msg.bounding_boxes[j].xmax) / 2 > basket[k].xmin - 5 &&
                            (msg.bounding_boxes[j].xmin + msg.bounding_boxes[j].xmax) / 2 < basket[k].xmax + 5 && 
                            (msg.bounding_boxes[j].ymin + msg.bounding_boxes[j].ymax) / 2 < basket[k].ymax + 10) {
                                // 消掉(msg.bounding_boxes[j].ymin + msg.bounding_boxes[j].ymax) / 2 > basket[i].ymin - 30 && 需要的效果 在逐渐靠进的过程中 不出问题
                                k++;
                                ball.push_back(msg.bounding_boxes[j]);
                                Sum_ball++;
                        }
                }
                std::sort(ball.begin(), ball.end(), Ball_Compare);

                Temp_arr[i][0] = Sum_ball;
                for (int m = 0; m < Sum_ball; ++m) {
                    if (ball[m].Class == "red") {
                        Temp_arr[i][m + 1] = 1;
                    } else {
                        Temp_arr[i][m + 1] = 2;
                    } 
                }
                Sum_ball = 0;
                ball.clear(); 
            }
            basket_rank.clear();
        }
        else if (Temp_basket < basket_rank[1] - basket_rank[0] + 1) {
            // 这里仍需要完成Temp_arr这个数组 需要搭建一个存储这个预测框数据结构的容器 这个可以大但不要太大
            /*
                int basket_world_location[5][2] = {375, 625,
                                   1125, 1375,
                                   1875, 2125,
                                   2625, 2875,
                                   3375, 3825};
                获取框的左右领界值初始化为零
                int Guest_arr[5][2] = {0};
            */
            double Temp_det = 640.0 / (Right_From_tuple - Life_From_tuple);

            for (int i = basket_rank[0] - 1; i < basket_rank[1]; ++i) {
                // 寻找问题
                Guest_arr[i][0] = (basket_world_location[i][0] - Life_From_tuple) * Temp_det - 5;
                Guest_arr[i][0] = (basket_world_location[i][1] - Life_From_tuple) * Temp_det + 5;
            }
            //* 得到一个从小到大框的容器
            for (int i = 0; i < num_thing; ++i) {
                if (msg.bounding_boxes[i].Class == "basket")
                    basket.push_back(msg.bounding_boxes[i]);
            }
            std::sort(basket.begin(), basket.end(), Bsk_Compare);
            std::array<int, 5> sorted = {0, 1, 2, 3, 4};
            std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
                return abs((basket_world_location[a][0] + basket_world_location[a][1]) / 2 - (Life_From_tuple + Right_From_tuple) / 2 ) < abs((basket_world_location[b][0] + basket_world_location[b][1]) / 2 - (Life_From_tuple + Right_From_tuple) / 2);});
            for (int q = 0; q < 5; ++q) 
                rank[q] = sorted[q] + 1;
            //* 我预测的要多于我识别出来的 绝对有预测出来的没有被YOLO认出来
            //* 这里主要是为了得到一个Temp_arr的容器
            for (int i = basket_rank[0] - 1, k = 0; i < basket_rank[1]; ++i) {
                if (Guest_arr[i][0] <= basket[k].xmin && Guest_arr[i][1] >= basket[k].xmax) {
                    for (int j = 0; j < num_thing; ++j) {
                            if (msg.bounding_boxes[j].Class != "basket" && 
                                (msg.bounding_boxes[j].xmin + msg.bounding_boxes[j].xmax) / 2 > basket[k].xmin - 5 &&
                                (msg.bounding_boxes[j].xmin + msg.bounding_boxes[j].xmax) / 2 < basket[k].xmax + 5 && 
                                (msg.bounding_boxes[j].ymin + msg.bounding_boxes[j].ymax) / 2 < basket[k].ymax + 10) {
                                    // 消掉(msg.bounding_boxes[j].ymin + msg.bounding_boxes[j].ymax) / 2 > basket[i].ymin - 30 &&
                                    // 需要的效果 在逐渐靠进的过程中 不出问题
                                    k++;
                                    ball.push_back(msg.bounding_boxes[j]);
                                    Sum_ball++;
                            }
                    }
                    std::sort(ball.begin(), ball.end(), Ball_Compare);
                    Temp_arr[i][0] = Sum_ball;
                    for (int m = 0; m < Sum_ball; ++m) {
                        if (ball[m].Class == "red") {
                            Temp_arr[i][m + 1] = 1;
                        } else {
                            Temp_arr[i][m + 1] = 2;
                        } 
                    }
                    Sum_ball = 0;
                    ball.clear(); 
                }
                else {
                    k++;
                }
            }
            // 从这里出来的容器一定会少于预测数量
            basket_rank.clear();
        }
        else {
            ROS_INFO("YOLO识别的框要多于预测的框 SB!!!!");
            basket_rank.clear();
            return;
        }

        //* 纠错容器构建***************************************************************************************************************************************
        /**
        *  1. 如果一定时间没有某一框的数据
        *  2. 如果一个框满了
        1. 少球大多数情况不予考虑 该情况的其他的框可以用做补充数据情况
        2. 多球的情况 大多数是一个球 极少情况是两个一起放 不可能有三球的情况 
        */
        /*
            int Temp_arr[5][4] = {0};
            for (int i = 0; i < 5; ++i)
                Temp_arr[i][0] = -1;
        */
       // 1. 如果数量是-1 那就先不考虑
       // 2. 少球的情况 这里要避免本身就少球
       // 
       // 3. 多球的情况 大多数是一个球 极少情况是两个一起放 不可能有三球的情况
       // Add_Ball_Num这个变量来观测
       // 4. 这里颜色可能会出现问题
       /**
        * 就是建立一个纠正机制
        * 尽可能的把一组数据中有问题的杀掉
        * 扰动  1. 真实 我方放了 对方放了 把握一个放球变化 这个arr数组要做到的第一点
        *      2. 虚假 yolo的识别问题 我们要滤掉一次数据集里那些虚假的数据
        *  
        *
        * 如果球的数量少了 需要一个纠的过程 不能太长了
        * 
        * 
        * 球的状态估计
        * 一个框里的球可能有实实在在的在里面
        * 机器人举着跑呢 (其实只要不是撒比 应该高于框 但如果低于框？？？)
        * 就是我的那个框里球的判断需要取一手 比例
        *
        *  
       */

        // 这里可以增添矫正功能
        for (int i = 0; i < 5; ++i) {
            if (New_Change == 1) {
                New_Change = 0;
                arr[ID_Now - 1][0]++;
                if (arr[ID_Now - 1][0] == 3) {
                    ROS_INFO("our put ball error");
                }
                for (int i = 1; i < 4; ++i) {
                    if (arr[ID_Now - 1][i] == 0) {
                        // 则为我方颜色的球
                        arr[ID_Now - 1][i] = 2;
                    }
                }
            }
            // 未观察到 或者 某些情况出现问题 完全可以过的
            if (Temp_arr[i][0] == -1) 
                continue;

            // 少球大多数是YOLO的问题(隔位少球) 但是不可避免有上次更新的就有问题 出现这种少球的情况大多数可以省略
            if (Temp_arr[i][0] < arr[i][0]) {
                // 最好不要发生什么
                // 这里可能会遇到的问题是 错误的把对面机器人的球归为框里的球
                if (arr[i][0] - Temp_arr[i][0] == 1) {
                    int Temp_DI_Dif = 0;
                    for (int j = 1; j < 4; ++j) {
                        if (arr[i][j] != Temp_arr[i][j] && arr[i][j] == red) {
                            Temp_DI_Dif++;
                        }
                        // 如果少的是对面的球
                       
                    }
                    if (Temp_DI_Dif == 1) {
                        VS_Robot[i]++;
                        if (VS_Robot[i] == 3) {
                            arr[i][0] = Temp_arr[i][0];
                            arr[i][1] = Temp_arr[i][1];
                            arr[i][2] = Temp_arr[i][2];
                            arr[i][3] = Temp_arr[i][3];
                            VS_Robot[i] = 0;
                        }
                        // else if () {
                        //     // 清零
                        // }
                    }
                }
                // if (arr[i][0] - Temp_arr[i][0] >= 2) {
                //     // 直接滚蛋就好
                // }
            }
            // 这里球的数目增多可能是 对面的球 放入框中  或是  YOLO 出现错误  或是   对面机器人举着呢 这三种情况的叠加
            // 这里跟更多应该考虑YOLO出错和标志位生效
            if (Temp_arr[i][0] > arr[i][0])
                Add_Ball_Num++;
            if (Add_Ball_Num > 1) {
                // 鸡
                // 在这里标志位高于一切
                ROS_INFO("Ball to mach");
                return;
            }
////////////////////////////////////// 这里要刷新Change_or_Not_Change 变量  切忌刷新的时候 只在对面球的时候刷新
//////////////////////////////////////          New_Change确保确实是改变了 用后清零 和 ID_Now 用后没必要清零 
        }

        // 记录出一个球的时候是两种情况 我这里需要决策的是对面的一球的情况 1. 确实放了 2. 对面机器人带着走 3. YOLO出现问题
        if (Add_Ball_Num == 1) {
            // 如果是我方的球我会增添一个标记 
            // 当上位机传来以放下球时 该标记生效
            /*
                if 标记存在
                且增加的为我方颜色的
                直接填上
            */
            // 这里必是对方的颜色的球
            // 这里叠加三次就够了
            // 更换完状态之后记得 清零int Dp_Dif[5] = {0};
            /**
            */
            for (int i = 0; i < 5; ++i) {
                if (Temp_arr[i][0] > arr[i][0]) {
                    Dp_Dif[i]++;
                    Big_Num--;
                }
                if (Big_Num == 0 && Dp_Dif[i] == 3) {
                    // 在3 帧图像中未出现明显的问题
                    // 数据的交换
                    arr[i][0] = Temp_arr[i][0];
                    arr[i][1] = Temp_arr[i][1];
                    arr[i][2] = Temp_arr[i][2];
                    arr[i][3] = Temp_arr[i][3];
                    Dp_Dif[i] = 0;
                    Big_Num = 3;
                }
                if (Big_Num < 0 && Dp_Dif[i] != 3) {
                    Dp_Dif[0] = 0;
                    Dp_Dif[1] = 0;
                    Dp_Dif[2] = 0;
                    Dp_Dif[3] = 0;
                    Dp_Dif[4] = 0;
                }
            }
        }
        // 这里会传出一个数据更新的标志位

    //** 在有5个框的时候构建框的容器
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
        ///////////////////////////////////////////////
        // Ha_num = into_box(arr, rank);
        // //* 对方投此框的函数
        // for (int e = 1; e < 4; ++e) {
        //     if (arr[Ha_num - 1][e] == 0) {
        //         arr[Ha_num - 1][0]++;
        //         arr[Ha_num - 1][e] = red;    
        //         break;
        //     }
        // }
        // ROS_INFO("strat");
        // for (int i = 0; i < 5; ++i) {
        //     for (int j = 0; j < 4; ++j) {
        //         ROS_INFO("%d", arr[i][j]);
        //     }
        //     ROS_INFO("\n");
        // }
        // ROS_INFO("end");
        // Ha_num *= 10;
        // Ha_num = Ha_num + into_box(arr, rank);
        // if (My_set.find(Ha_num) == My_set.end()) {
        //     My_set.insert(Ha_num);
        //     My_map.emplace(Ha_num, 1);
        //     Arv_basket++;
        // }
        // else if (My_set.find(Ha_num) != My_set.end()) {
        //     My_map[Ha_num]++;
        //     Arv_basket++;
        // }
        // if (Arv_basket >= 100) {
        //     int Temp{};
        //         for (int num : My_set) {
        //             if (My_map[num] >= Temp) {
        //                 Temp = My_map[num];
        //                 Send_num = num;
        //             }
        //         }
        //     //mode = 0;
        //     ROS_INFO("%d", Send_num);
        // }
        //////////////////////////***********************************************************************************************************************************

        basket.clear();
        }
} 


//* 这里需要一个坐标数据和角度 只需给全局变量把值赋出去就可以
void TD_location(const yolov5_ros_msgs::X_Y_ARG & msg) {
    // 这里的值是一直在改变的
    // 这个线程是必须的
    static ros::NodeHandle nh;
    static ros::Publisher message_pub = nh.advertise<yolov5_ros_msgs::port_serial>("basket_id", 10);
    static yolov5_ros_msgs::port_serial Bask_ID;
    if (New_Get == 0) {
        // 这里问题不大
        New_X = msg.x;
        New_Y = msg.y;
        New_Arg = msg.arg;
        //ROS_INFO("%d %d %d\n", New_X, New_Y, New_Arg);
        //ROS_INFO("jiao jiao queen !!!");
    }

    if (msg.mode == 1) {
        // 一但发送数据非必要不变原则 就是对面的投了球  
        // Change_or_Not_Change 是用来
        if (Change_or_Not_Change == 0) {
            ID_Now = into_box(arr, rank);
            Change_or_Not_Change = 1;
        }
        // 对面没投球需要决策 
        Bask_ID.id = ID_Now;
        message_pub.publish(Bask_ID);
        IS_IS_OK = 1;
    }
    // 放球了入框了
    if (msg.mode == 2) {
        // 标记位生效 并且保留投入的框的序号
        // 完成了投球后 请保留ID 以及 标志位
        if (IS_IS_OK == 1) {
            IS_IS_OK = 0;
            New_Change = 1;
            // ID_Now 就是我保留的全局ID
        }
    }
}

//* 主函数构建完成
int main(int argc , char *argv[]) {
    setlocale(LC_ALL , " " );
    ros::init(argc , argv , "message");
    ros::NodeHandle sh;
    ros::Subscriber message_sub = sh.subscribe("/yolov5/BoundingBoxes" , 3 , message_callback);
    ros::Subscriber location = sh.subscribe("X_Y_ARG", 1, TD_location);
    //* 这里是否使用循环多线程
    ros::MultiThreadedSpinner spinner(2);
    spinner.spin();
    return 0;
}