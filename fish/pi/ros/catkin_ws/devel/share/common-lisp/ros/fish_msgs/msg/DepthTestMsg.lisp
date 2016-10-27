; Auto-generated. Do not edit!


(cl:in-package fish_msgs-msg)


;//! \htmlinclude DepthTestMsg.msg.html

(cl:defclass <DepthTestMsg> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:float
    :initform 0.0))
)

(cl:defclass DepthTestMsg (<DepthTestMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DepthTestMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DepthTestMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fish_msgs-msg:<DepthTestMsg> is deprecated: use fish_msgs-msg:DepthTestMsg instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <DepthTestMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:mode-val is deprecated.  Use fish_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <DepthTestMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:value-val is deprecated.  Use fish_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DepthTestMsg>) ostream)
  "Serializes a message object of type '<DepthTestMsg>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DepthTestMsg>) istream)
  "Deserializes a message object of type '<DepthTestMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'value) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DepthTestMsg>)))
  "Returns string type for a message object of type '<DepthTestMsg>"
  "fish_msgs/DepthTestMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DepthTestMsg)))
  "Returns string type for a message object of type 'DepthTestMsg"
  "fish_msgs/DepthTestMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DepthTestMsg>)))
  "Returns md5sum for a message object of type '<DepthTestMsg>"
  "392256b70b5280d59063ab9c2ddc1efa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DepthTestMsg)))
  "Returns md5sum for a message object of type 'DepthTestMsg"
  "392256b70b5280d59063ab9c2ddc1efa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DepthTestMsg>)))
  "Returns full string definition for message of type '<DepthTestMsg>"
  (cl:format cl:nil "int8 mode~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DepthTestMsg)))
  "Returns full string definition for message of type 'DepthTestMsg"
  (cl:format cl:nil "int8 mode~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DepthTestMsg>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DepthTestMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'DepthTestMsg
    (cl:cons ':mode (mode msg))
    (cl:cons ':value (value msg))
))
