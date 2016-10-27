; Auto-generated. Do not edit!


(cl:in-package fish_msgs-msg)


;//! \htmlinclude FishCtrlMsg.msg.html

(cl:defclass <FishCtrlMsg> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:fixnum
    :initform 0)
   (thrust
    :reader thrust
    :initarg :thrust
    :type cl:float
    :initform 0.0)
   (dvalue
    :reader dvalue
    :initarg :dvalue
    :type cl:float
    :initform 0.0)
   (yaw
    :reader yaw
    :initarg :yaw
    :type cl:float
    :initform 0.0)
   (valve
    :reader valve
    :initarg :valve
    :type cl:float
    :initform 0.0))
)

(cl:defclass FishCtrlMsg (<FishCtrlMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FishCtrlMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FishCtrlMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fish_msgs-msg:<FishCtrlMsg> is deprecated: use fish_msgs-msg:FishCtrlMsg instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <FishCtrlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:mode-val is deprecated.  Use fish_msgs-msg:mode instead.")
  (mode m))

(cl:ensure-generic-function 'thrust-val :lambda-list '(m))
(cl:defmethod thrust-val ((m <FishCtrlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:thrust-val is deprecated.  Use fish_msgs-msg:thrust instead.")
  (thrust m))

(cl:ensure-generic-function 'dvalue-val :lambda-list '(m))
(cl:defmethod dvalue-val ((m <FishCtrlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:dvalue-val is deprecated.  Use fish_msgs-msg:dvalue instead.")
  (dvalue m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <FishCtrlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:yaw-val is deprecated.  Use fish_msgs-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'valve-val :lambda-list '(m))
(cl:defmethod valve-val ((m <FishCtrlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fish_msgs-msg:valve-val is deprecated.  Use fish_msgs-msg:valve instead.")
  (valve m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FishCtrlMsg>) ostream)
  "Serializes a message object of type '<FishCtrlMsg>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'thrust))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'dvalue))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'valve))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FishCtrlMsg>) istream)
  "Deserializes a message object of type '<FishCtrlMsg>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'thrust) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dvalue) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'yaw) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'valve) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FishCtrlMsg>)))
  "Returns string type for a message object of type '<FishCtrlMsg>"
  "fish_msgs/FishCtrlMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FishCtrlMsg)))
  "Returns string type for a message object of type 'FishCtrlMsg"
  "fish_msgs/FishCtrlMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FishCtrlMsg>)))
  "Returns md5sum for a message object of type '<FishCtrlMsg>"
  "9305e87d9033eb2ef9f1c3927209a31b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FishCtrlMsg)))
  "Returns md5sum for a message object of type 'FishCtrlMsg"
  "9305e87d9033eb2ef9f1c3927209a31b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FishCtrlMsg>)))
  "Returns full string definition for message of type '<FishCtrlMsg>"
  (cl:format cl:nil "int8 mode~%float32 thrust~%float32 dvalue~%float32 yaw~%float32 valve~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FishCtrlMsg)))
  "Returns full string definition for message of type 'FishCtrlMsg"
  (cl:format cl:nil "int8 mode~%float32 thrust~%float32 dvalue~%float32 yaw~%float32 valve~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FishCtrlMsg>))
  (cl:+ 0
     1
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FishCtrlMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'FishCtrlMsg
    (cl:cons ':mode (mode msg))
    (cl:cons ':thrust (thrust msg))
    (cl:cons ':dvalue (dvalue msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':valve (valve msg))
))
