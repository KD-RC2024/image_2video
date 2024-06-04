#include "ros/ros.h"
#include "std_msgs/String.h"
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/BoundingBoxes.h"
#include "vector"
//#include "unordered_map"
#include "algorithm"
//#include "string"
//判断是否在其中

std::vector<yolov5_ros_msgs::BoundingBox> basket(15);
std::vector<yolov5_ros_msgs::BoundingBox> ballR(18);
std::vector<yolov5_ros_msgs::BoundingBox> ballB(18);
std::vector<yolov5_ros_msgs::BoundingBox> bal_in_box(9);
std::vector<int64_t> i_y;
bool My_Is_Not(yolov5_ros_msgs::BoundingBox &bas, yolov5_ros_msgs::BoundingBox &bal) {
    return bas.xmax + 5 > bal.xmax && bas.ymax + 5 > bal.ymax && bas.ymin - 40 < bal.ymin && bas.xmin - 5 < bal.xmin;
}

bool Basket_Compare(yolov5_ros_msgs::BoundingBox &a , yolov5_ros_msgs::BoundingBox &b ){
    return a.ymin < b.ymin ;
}

bool Basket_CompareK(yolov5_ros_msgs::BoundingBox &a , yolov5_ros_msgs::BoundingBox &b ){
    return a.xmin < b.xmin ;
}

//完成arr[][]的数据的调用函数
std::vector<int64_t> Sort(std::vector<yolov5_ros_msgs::BoundingBox> & a) {
    //传入的是一个框内的球的数据
    std::vector<int64_t> arr_1(4,0);
    arr_1[0] = a.size();
    if (arr_1[0] == 0) {
        return arr_1;
    } else {
        if (arr_1[0] > 1)
           sort(a.begin(),a.end(),Basket_Compare);
        for (int i = 0; i < arr_1[0]; ++i) {
            if (a[i].Class == "red") {
                arr_1[i + 1] = 1;
            } else if (a[i].Class == "blue") {
                arr_1[i + 1] = 2;
            } 
        }
    }
    return arr_1;
}

//接收数据处理的回调函数
//const yolov5_ros_msgs::BoundingBoxes::ConstPtr &msg
void message_callback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr & msg) {  //::ConstPtr
    //一张图片的对象
    int16_t Bas_num {};
    int16_t Red_num {};
    int16_t Blue_num {};
    if (!msg->bounding_boxes.empty()) {
    int arr[5][4] = {0};
    int16_t recognize_sum = msg->bounding_boxes[0].num;
    //完成数据类型的初始化basket ballB ballR
    for (int i = 0; i < recognize_sum; ++i) {
        if (msg->bounding_boxes[i].Class == "basket") {
            basket[Bas_num] = msg->bounding_boxes[i];
            Bas_num++;
        //这里保留分开的红篮球数
        } else if (msg->bounding_boxes[i].Class == "blue") {
            ballB[Blue_num] = msg->bounding_boxes[i];
            Blue_num++;
        } else if (msg->bounding_boxes[i].Class == "red"){
            ballR[Red_num] = msg->bounding_boxes[i];
            Red_num++;
        }    
    }
    sort(basket.begin(), basket.end(), Basket_CompareK);
    //完成arr[][]的数据
    if (Bas_num == 5) {
    for (int i = 0; i < Bas_num; ++i) {
        //Sort(std::vector<yolov5_ros_msgs::BoundingBox> & a)
        for (int j = 0; j < Blue_num; ++j) {
            if ( My_Is_Not(basket[i], ballR[j]) ) {
                bal_in_box.push_back(ballR[j]);
            }
        }
        for (int j = 0; j < Red_num; ++j) {
            if ( My_Is_Not(basket[i], ballB[j]) ) {
                bal_in_box.push_back(ballB[j]);
            }
        }

        i_y = Sort(bal_in_box);
        std::copy(i_y.begin(), i_y.end(), arr[i]);
        if (!bal_in_box.empty())
            bal_in_box.clear();
        if (!i_y.empty())
            i_y.clear();
        

    }
    //完成与yolo_v5.py节点的交互 
        for (int i = 0; i < Bas_num; ++i) {
            for (int j = 0; j < 4; ++j) {
            ROS_INFO("%d", arr[i][j]);
            }
            ROS_INFO("\n");
        }
        ROS_INFO("99999");
        ros::Rate rate(1);
        rate.sleep();
    }
    }
}

int main(int argc , char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc , argv , "message_send_node");
    ros::NodeHandle sh;
    ros::Subscriber message_sub = sh.subscribe<yolov5_ros_msgs::BoundingBoxes>("pub_topicK", 10, message_callback);
    ROS_INFO("oooooo");
    ros::spin();
    return 0;
}