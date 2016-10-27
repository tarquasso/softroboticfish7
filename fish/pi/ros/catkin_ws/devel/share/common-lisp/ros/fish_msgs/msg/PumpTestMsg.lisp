; Auto-generated. Do not edit!


(cl:in-package fish_msgs-msg)


;//! \htmlinclude PumpTestMsg.msg.html

(cl:defclass <PumpTestMsg> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (freq
    :reader freq
    :initarg :freq
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass PumpTestMsg (<PumpTestMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PumpTestMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PumpTestMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fish_msgs-msg:<PumpTestMsg> is deprecated: use fish_msgs-msg:PumpTestMsg instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <PumpTestMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:mode-val is deprecated.  Use fish_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'freq-val :lambda-list '(m))
(cl:defmethod freq-val ((m <PumpTestMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:freq-val is deprecated.  Use fish_msgs-msg:freq instead.")
  (freq m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <PumpTestMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:yaw-val is deprecated.  Use fish_msgs-msg:yaw instead.")
  (yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PumpTestMsg>) ostream)
  "Serializes a message object of type '<PumpTestMsg>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'freq))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PumpTestMsg>) istream)
  "Deserializes a message object of type '<PumpTestMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'freq) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PumpTestMsg>)))
  "Returns string type for a message object of type '<PumpTestMsg>"
  "fish_msgs/PumpTestMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PumpTestMsg)))
  "Returns string type for a message object of type 'PumpTestMsg"
  "fish_msgs/PumpTestMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PumpTestMsg>)))
  "Returns md5sum for a message object of type '<PumpTestMsg>"
  "34fadd9771870a86de960e911d1d8c17")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PumpTestMsg)))
  "Returns md5sum for a message object of type 'PumpTestMsg"
  "34fadd9771870a86de960e911d1d8c17")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PumpTestMsg>)))
  "Returns full string definition for message of type '<PumpTestMsg>"
  (cl:format cl:nil "int8 mode~%float32 freq~%float32 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PumpTestMsg)))
  "Returns full string definition for message of type 'PumpTestMsg"
  (cl:format cl:nil "int8 mode~%float32 freq~%float32 yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PumpTestMsg>))
  (cl:+ 0
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PumpTestMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'PumpTestMsg
    (cl:cons ':mode (mode msg))
    (cl:cons ':freq (freq msg))
    (cl:cons ':yaw (yaw msg))
))
