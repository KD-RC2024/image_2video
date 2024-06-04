;; Auto-generated. Do not edit!


(when (boundp 'yolov5_ros_msgs::port_serial)
  (if (not (find-package "YOLOV5_ROS_MSGS"))
    (make-package "YOLOV5_ROS_MSGS"))
  (shadow 'port_serial (find-package "YOLOV5_ROS_MSGS")))
(unless (find-package "YOLOV5_ROS_MSGS::PORT_SERIAL")
  (make-package "YOLOV5_ROS_MSGS::PORT_SERIAL"))

(in-package "ROS")
;;//! \htmlinclude port_serial.msg.html


(defclass yolov5_ros_msgs::port_serial
  :super ros::object
  :slots (_id _x _y _distance ))

(defmethod yolov5_ros_msgs::port_serial
  (:init
   (&key
    ((:id __id) 0)
    ((:x __x) 0)
    ((:y __y) 0)
    ((:distance __distance) 0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _distance (round __distance))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; int32 _x
    4
    ;; int32 _y
    4
    ;; int32 _distance
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;; int32 _distance
       (write-long _distance s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _distance
     (setq _distance (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yolov5_ros_msgs::port_serial :md5sum-) "2b3c09fcdbc0402971b87d1180f413f1")
(setf (get yolov5_ros_msgs::port_serial :datatype-) "yolov5_ros_msgs/port_serial")
(setf (get yolov5_ros_msgs::port_serial :definition-)
      "int32 id
int32 x
int32 y
int32 distance
")



(provide :yolov5_ros_msgs/port_serial "2b3c09fcdbc0402971b87d1180f413f1")


