; Auto-generated. Do not edit!


(cl:in-package yolov5_ros_msgs-msg)


;//! \htmlinclude X_Y_ARG.msg.html

(cl:defclass <X_Y_ARG> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0)
   (arg
    :reader arg
    :initarg :arg
    :type cl:fixnum
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass X_Y_ARG (<X_Y_ARG>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <X_Y_ARG>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'X_Y_ARG)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yolov5_ros_msgs-msg:<X_Y_ARG> is deprecated: use yolov5_ros_msgs-msg:X_Y_ARG instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <X_Y_ARG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolov5_ros_msgs-msg:x-val is deprecated.  Use yolov5_ros_msgs-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <X_Y_ARG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolov5_ros_msgs-msg:y-val is deprecated.  Use yolov5_ros_msgs-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'arg-val :lambda-list '(m))
(cl:defmethod arg-val ((m <X_Y_ARG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolov5_ros_msgs-msg:arg-val is deprecated.  Use yolov5_ros_msgs-msg:arg instead.")
  (arg m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <X_Y_ARG>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yolov5_ros_msgs-msg:mode-val is deprecated.  Use yolov5_ros_msgs-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <X_Y_ARG>) ostream)
  "Serializes a message object of type '<X_Y_ARG>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'arg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <X_Y_ARG>) istream)
  "Deserializes a message object of type '<X_Y_ARG>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'arg) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<X_Y_ARG>)))
  "Returns string type for a message object of type '<X_Y_ARG>"
  "yolov5_ros_msgs/X_Y_ARG")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'X_Y_ARG)))
  "Returns string type for a message object of type 'X_Y_ARG"
  "yolov5_ros_msgs/X_Y_ARG")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<X_Y_ARG>)))
  "Returns md5sum for a message object of type '<X_Y_ARG>"
  "3221f6e0c05aea0457e6a44220fa22a2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'X_Y_ARG)))
  "Returns md5sum for a message object of type 'X_Y_ARG"
  "3221f6e0c05aea0457e6a44220fa22a2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<X_Y_ARG>)))
  "Returns full string definition for message of type '<X_Y_ARG>"
  (cl:format cl:nil "int16 x~%int16 y~%int16 arg~%int16 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'X_Y_ARG)))
  "Returns full string definition for message of type 'X_Y_ARG"
  (cl:format cl:nil "int16 x~%int16 y~%int16 arg~%int16 mode~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <X_Y_ARG>))
  (cl:+ 0
     2
     2
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <X_Y_ARG>))
  "Converts a ROS message object to a list"
  (cl:list 'X_Y_ARG
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':arg (arg msg))
    (cl:cons ':mode (mode msg))
))
