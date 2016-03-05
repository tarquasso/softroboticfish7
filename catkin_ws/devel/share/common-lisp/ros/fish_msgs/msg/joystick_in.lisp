; Auto-generated. Do not edit!


(cl:in-package fish_msgs-msg)


;//! \htmlinclude joystick_in.msg.html

(cl:defclass <joystick_in> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass joystick_in (<joystick_in>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <joystick_in>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'joystick_in)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fish_msgs-msg:<joystick_in> is deprecated: use fish_msgs-msg:joystick_in instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <joystick_in>) ostream)
  "Serializes a message object of type '<joystick_in>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <joystick_in>) istream)
  "Deserializes a message object of type '<joystick_in>"
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
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'joystick_in)))
  "Returns md5sum for a message object of type 'joystick_in"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<joystick_in>)))
  "Returns full string definition for message of type '<joystick_in>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'joystick_in)))
  "Returns full string definition for message of type 'joystick_in"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <joystick_in>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <joystick_in>))
  "Converts a ROS message object to a list"
  (cl:list 'joystick_in
))
