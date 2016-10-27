; Auto-generated. Do not edit!


(cl:in-package fish_msgs-msg)


;//! \htmlinclude mbedStatusMsg.msg.html

(cl:defclass <mbedStatusMsg> (roslisp-msg-protocol:ros-message)
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

(cl:defclass mbedStatusMsg (<mbedStatusMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <mbedStatusMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'mbedStatusMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fish_msgs-msg:<mbedStatusMsg> is deprecated: use fish_msgs-msg:mbedStatusMsg instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <mbedStatusMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:mode-val is deprecated.  Use fish_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <mbedStatusMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:value-val is deprecated.  Use fish_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <mbedStatusMsg>) ostream)
  "Serializes a message object of type '<mbedStatusMsg>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <mbedStatusMsg>) istream)
  "Deserializes a message object of type '<mbedStatusMsg>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<mbedStatusMsg>)))
  "Returns string type for a message object of type '<mbedStatusMsg>"
  "fish_msgs/mbedStatusMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'mbedStatusMsg)))
  "Returns string type for a message object of type 'mbedStatusMsg"
  "fish_msgs/mbedStatusMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<mbedStatusMsg>)))
  "Returns md5sum for a message object of type '<mbedStatusMsg>"
  "392256b70b5280d59063ab9c2ddc1efa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'mbedStatusMsg)))
  "Returns md5sum for a message object of type 'mbedStatusMsg"
  "392256b70b5280d59063ab9c2ddc1efa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<mbedStatusMsg>)))
  "Returns full string definition for message of type '<mbedStatusMsg>"
  (cl:format cl:nil "int8 mode~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'mbedStatusMsg)))
  "Returns full string definition for message of type 'mbedStatusMsg"
  (cl:format cl:nil "int8 mode~%float32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <mbedStatusMsg>))
  (cl:+ 0
     1
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <mbedStatusMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'mbedStatusMsg
    (cl:cons ':mode (mode msg))
    (cl:cons ':value (value msg))
))
