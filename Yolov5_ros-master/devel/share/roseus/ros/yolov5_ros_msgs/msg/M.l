;; Auto-generated. Do not edit!


(when (boundp 'yolov5_ros_msgs::M)
  (if (not (find-package "YOLOV5_ROS_MSGS"))
    (make-package "YOLOV5_ROS_MSGS"))
  (shadow 'M (find-package "YOLOV5_ROS_MSGS")))
(unless (find-package "YOLOV5_ROS_MSGS::M")
  (make-package "YOLOV5_ROS_MSGS::M"))

(in-package "ROS")
;;//! \htmlinclude M.msg.html


(defclass yolov5_ros_msgs::M
  :super ros::object
  :slots (_arr1 _arr2 _arr3 _arr4 _arr5 _num ))

(defmethod yolov5_ros_msgs::M
  (:init
   (&key
    ((:arr1 __arr1) (make-array 0 :initial-element 0 :element-type :integer))
    ((:arr2 __arr2) (make-array 0 :initial-element 0 :element-type :integer))
    ((:arr3 __arr3) (make-array 0 :initial-element 0 :element-type :integer))
    ((:arr4 __arr4) (make-array 0 :initial-element 0 :element-type :integer))
    ((:arr5 __arr5) (make-array 0 :initial-element 0 :element-type :integer))
    ((:num __num) 0)
    )
   (send-super :init)
   (setq _arr1 __arr1)
   (setq _arr2 __arr2)
   (setq _arr3 __arr3)
   (setq _arr4 __arr4)
   (setq _arr5 __arr5)
   (setq _num (round __num))
   self)
  (:arr1
   (&optional __arr1)
   (if __arr1 (setq _arr1 __arr1)) _arr1)
  (:arr2
   (&optional __arr2)
   (if __arr2 (setq _arr2 __arr2)) _arr2)
  (:arr3
   (&optional __arr3)
   (if __arr3 (setq _arr3 __arr3)) _arr3)
  (:arr4
   (&optional __arr4)
   (if __arr4 (setq _arr4 __arr4)) _arr4)
  (:arr5
   (&optional __arr5)
   (if __arr5 (setq _arr5 __arr5)) _arr5)
  (:num
   (&optional __num)
   (if __num (setq _num __num)) _num)
  (:serialization-length
   ()
   (+
    ;; int32[] _arr1
    (* 4    (length _arr1)) 4
    ;; int32[] _arr2
    (* 4    (length _arr2)) 4
    ;; int32[] _arr3
    (* 4    (length _arr3)) 4
    ;; int32[] _arr4
    (* 4    (length _arr4)) 4
    ;; int32[] _arr5
    (* 4    (length _arr5)) 4
    ;; int32 _num
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _arr1
     (write-long (length _arr1) s)
     (dotimes (i (length _arr1))
       (write-long (elt _arr1 i) s)
       )
     ;; int32[] _arr2
     (write-long (length _arr2) s)
     (dotimes (i (length _arr2))
       (write-long (elt _arr2 i) s)
       )
     ;; int32[] _arr3
     (write-long (length _arr3) s)
     (dotimes (i (length _arr3))
       (write-long (elt _arr3 i) s)
       )
     ;; int32[] _arr4
     (write-long (length _arr4) s)
     (dotimes (i (length _arr4))
       (write-long (elt _arr4 i) s)
       )
     ;; int32[] _arr5
     (write-long (length _arr5) s)
     (dotimes (i (length _arr5))
       (write-long (elt _arr5 i) s)
       )
     ;; int32 _num
       (write-long _num s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _arr1
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _arr1 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _arr1 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _arr2
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _arr2 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _arr2 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _arr3
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _arr3 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _arr3 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _arr4
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _arr4 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _arr4 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _arr5
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _arr5 (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _arr5 i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32 _num
     (setq _num (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yolov5_ros_msgs::M :md5sum-) "02c60c121c0de351ab241d16cef9d248")
(setf (get yolov5_ros_msgs::M :datatype-) "yolov5_ros_msgs/M")
(setf (get yolov5_ros_msgs::M :definition-)
      "int32[] arr1
int32[] arr2
int32[] arr3
int32[] arr4
int32[] arr5
int32 num
")



(provide :yolov5_ros_msgs/M "02c60c121c0de351ab241d16cef9d248")


