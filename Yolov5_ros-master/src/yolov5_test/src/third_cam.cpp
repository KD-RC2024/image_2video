#include <ros/ros.h>
#include <std_msgs/String.h>
#include <algorithm>
#include "yolov5_ros_msgs/BoundingBoxes.h"
#include "yolov5_ros_msgs/BoundingBox.h"
#include "yolov5_ros_msgs/port_serial.h"
#include <opencv2/opencv.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#define our_ball_name "blue"
#define Low_Pass_Filter 0.6

static yolov5_ros_msgs::port_serial target_ball;
static yolov5_ros_msgs::port_serial middle_ball;
typedef struct {
	uint32_t weight;
    uint8_t id;
    uint32_t distance;
}Basket_Typedef;


//距离比较
bool Our_ball_Compare(const yolov5_ros_msgs::BoundingBox & a , const yolov5_ros_msgs::BoundingBox &b) {
    return Low_Pass_Filter* ( ( pow((a.xmin + a.xmax)/2 - 320,2) + pow((a.ymax + a.ymin )/2 - 480 , 2)  )  )+ 
    (1- Low_Pass_Filter) * ( pow(((a.xmin + a.xmax )/2 - middle_ball.x ) , 2 ) + pow(((a.ymin+a.ymax)/2 - middle_ball.y),2) )<
    Low_Pass_Filter * (( pow((b.xmin + b.xmax)/2 - 320,2) + pow((b.ymax + b.ymin )/2 - 480 , 2)  ) )+
    (1- Low_Pass_Filter) * (pow(((b.xmin + b.xmax )/2 - middle_ball.x ) , 2 ) + pow(((b.ymin+b.ymax)/2 - middle_ball.y),2) ) ;
}
void message_callback(const yolov5_ros_msgs::BoundingBoxes & msg){

    int16_t sum = msg.bounding_boxes[0].num;
    int16_t basket_num = 0 , ball_num = 0 , our_ball_num = 0;

    yolov5_ros_msgs::BoundingBoxes basket_array;
    yolov5_ros_msgs::BoundingBoxes ball_array;
    yolov5_ros_msgs::BoundingBoxes our_ball_array = {};

    static ros::NodeHandle nh;
    static ros::Publisher message_pub = nh.advertise< yolov5_ros_msgs::port_serial>("basket_id", 10);

    if (msg.bounding_boxes.size() == msg.bounding_boxes[0].num) {

    for (int i = 0 , basket_num = 0 ;  i < msg.bounding_boxes[0].num ; ++i) {
        if (msg.bounding_boxes[i].Class == "basket") {
            basket_array.bounding_boxes.push_back(msg.bounding_boxes[i]);
            basket_num ++ ;
        }
        if (msg.bounding_boxes[i].Class == our_ball_name ) {
            our_ball_array.bounding_boxes.push_back(msg.bounding_boxes[i]);
            our_ball_num ++ ;
        }
    }

    if (our_ball_num  == 0 || sum == 0 ) { 
        middle_ball.x = 320; 
        middle_ball.y = 480; 
        middle_ball.distance = 0; 
        middle_ball.id = 1;
        }
    else {
    std::sort(our_ball_array.bounding_boxes.begin() , our_ball_array.bounding_boxes.end() , Our_ball_Compare );
    
    if(our_ball_num) {
    middle_ball.x = (int)round((our_ball_array.bounding_boxes[0].xmin + our_ball_array.bounding_boxes[0].xmax)/2);
    middle_ball.y = (int)round((our_ball_array.bounding_boxes[0].ymin + our_ball_array.bounding_boxes[0].ymax)/2);
    target_ball.x = round ( ( our_ball_array.bounding_boxes[0].xmax + our_ball_array.bounding_boxes[0].xmin )/2  ) - 320;
    target_ball.y =   (240 - round ( ( our_ball_array.bounding_boxes[0].ymax + our_ball_array.bounding_boxes[0].ymin )/2  ));
    target_ball.distance = round ( sqrt ( pow((our_ball_array.bounding_boxes[0].xmin + our_ball_array.bounding_boxes[0].xmax)/2 - 320,2) + 
    pow((our_ball_array.bounding_boxes[0].ymax + our_ball_array.bounding_boxes[0].ymin )/2 - 240 , 2 )  )); 
            
    message_pub.publish(target_ball);
    }
}
    }
}

int main(int argc , char *argv[]) {
    setlocale(LC_ALL , "" );
    ros::init(argc , argv , "message_send_node");
    ros::NodeHandle sh;

    ros::Subscriber message_sub = sh.subscribe("/yolov5/BoundingBoxes" , 10 , message_callback);
    middle_ball.x = 320 ;
    middle_ball.y = 480 ;
    middle_ball.distance = 0 ;
    middle_ball.id = 0 ;
    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}