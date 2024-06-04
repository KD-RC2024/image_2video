;; Auto-generated. Do not edit!


(when (boundp 'yolov5_ros_msgs::X_Y_ARG)
  (if (not (find-package "YOLOV5_ROS_MSGS"))
    (make-package "YOLOV5_ROS_MSGS"))
  (shadow 'X_Y_ARG (find-package "YOLOV5_ROS_MSGS")))
(unless (find-package "YOLOV5_ROS_MSGS::X_Y_ARG")
  (make-package "YOLOV5_ROS_MSGS::X_Y_ARG"))

(in-package "ROS")
;;//! \htmlinclude X_Y_ARG.msg.html


(defclass yolov5_ros_msgs::X_Y_ARG
  :super ros::object
  :slots (_x _y _arg _mode ))

(defmethod yolov5_ros_msgs::X_Y_ARG
  (:init
   (&key
    ((:x __x) 0)
    ((:y __y) 0)
    ((:arg __arg) 0)
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _x (round __x))
   (setq _y (round __y))
   (setq _arg (round __arg))
   (setq _mode (round __mode))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:arg
   (&optional __arg)
   (if __arg (setq _arg __arg)) _arg)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; int16 _x
    2
    ;; int16 _y
    2
    ;; int16 _arg
    2
    ;; int16 _mode
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _x
       (write-word _x s)
     ;; int16 _y
       (write-word _y s)
     ;; int16 _arg
       (write-word _arg s)
     ;; int16 _mode
       (write-word _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _x
     (setq _x (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _y
     (setq _y (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _arg
     (setq _arg (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _mode
     (setq _mode (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get yolov5_ros_msgs::X_Y_ARG :md5sum-) "3221f6e0c05aea0457e6a44220fa22a2")
(setf (get yolov5_ros_msgs::X_Y_ARG :datatype-) "yolov5_ros_msgs/X_Y_ARG")
(setf (get yolov5_ros_msgs::X_Y_ARG :definition-)
      "int16 x
int16 y
int16 arg
int16 mode
")



(provide :yolov5_ros_msgs/X_Y_ARG "3221f6e0c05aea0457e6a44220fa22a2")


