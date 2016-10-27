; Auto-generated. Do not edit!


(cl:in-package fish_msgs-msg)


;//! \htmlinclude joystick_in.msg.html

(cl:defclass <joystick_in> (roslisp-msg-protocol:ros-message)
  ((freq_ctrl
    :reader freq_ctrl
    :initarg :freq_ctrl
    :type cl:fixnum
    :initform 0)
   (speed_ctrl
    :reader speed_ctrl
    :initarg :speed_ctrl
    :type cl:fixnum
    :initform 0)
   (depth_ctrl
    :reader depth_ctrl
    :initarg :depth_ctrl
    :type cl:fixnum
    :initform 0)
   (yaw_ctrl
    :reader yaw_ctrl
    :initarg :yaw_ctrl
    :type cl:fixnum
    :initform 0))
)

(cl:defclass joystick_in (<joystick_in>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joystick_in>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joystick_in)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fish_msgs-msg:<joystick_in> is deprecated: use fish_msgs-msg:joystick_in instead.")))

(cl:ensure-generic-function 'freq_ctrl-val :lambda-list '(m))
(cl:defmethod freq_ctrl-val ((m <joystick_in>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:freq_ctrl-val is deprecated.  Use fish_msgs-msg:freq_ctrl instead.")
  (freq_ctrl m))

(cl:ensure-generic-function 'speed_ctrl-val :lambda-list '(m))
(cl:defmethod speed_ctrl-val ((m <joystick_in>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:speed_ctrl-val is deprecated.  Use fish_msgs-msg:speed_ctrl instead.")
  (speed_ctrl m))

(cl:ensure-generic-function 'depth_ctrl-val :lambda-list '(m))
(cl:defmethod depth_ctrl-val ((m <joystick_in>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:depth_ctrl-val is deprecated.  Use fish_msgs-msg:depth_ctrl instead.")
  (depth_ctrl m))

(cl:ensure-generic-function 'yaw_ctrl-val :lambda-list '(m))
(cl:defmethod yaw_ctrl-val ((m <joystick_in>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:yaw_ctrl-val is deprecated.  Use fish_msgs-msg:yaw_ctrl instead.")
  (yaw_ctrl m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joystick_in>) ostream)
  "Serializes a message object of type '<joystick_in>"
  (cl:let* ((signed (cl:slot-value msg 'freq_ctrl)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed_ctrl)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'depth_ctrl)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'yaw_ctrl)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joystick_in>) istream)
  "Deserializes a message object of type '<joystick_in>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'freq_ctrl) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed_ctrl) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'depth_ctrl) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'yaw_ctrl) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<joystick_in>)))
  "Returns string type for a message object of type '<joystick_in>"
  "fish_msgs/joystick_in")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'joystick_in)))
  "Returns string type for a message object of type 'joystick_in"
  "fish_msgs/joystick_in")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<joystick_in>)))
  "Returns md5sum for a message object of type '<joystick_in>"
  "0c650c89727301b1e2298a1c19175b51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joystick_in)))
  "Returns md5sum for a message object of type 'joystick_in"
  "0c650c89727301b1e2298a1c19175b51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joystick_in>)))
  "Returns full string definition for message of type '<joystick_in>"
  (cl:format cl:nil "int8 freq_ctrl~%int8 speed_ctrl~%int8 depth_ctrl~%int8 yaw_ctrl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joystick_in)))
  "Returns full string definition for message of type 'joystick_in"
  (cl:format cl:nil "int8 freq_ctrl~%int8 speed_ctrl~%int8 depth_ctrl~%int8 yaw_ctrl~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joystick_in>))
  (cl:+ 0
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joystick_in>))
  "Converts a ROS message object to a list"
  (cl:list 'joystick_in
    (cl:cons ':freq_ctrl (freq_ctrl msg))
    (cl:cons ':speed_ctrl (speed_ctrl msg))
    (cl:cons ':depth_ctrl (depth_ctrl msg))
    (cl:cons ':yaw_ctrl (yaw_ctrl msg))
))
