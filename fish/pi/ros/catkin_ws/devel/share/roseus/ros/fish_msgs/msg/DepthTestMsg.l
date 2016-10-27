;; Auto-generated. Do not edit!


(when (boundp 'fish_msgs::DepthTestMsg)
  (if (not (find-package "FISH_MSGS"))
    (make-package "FISH_MSGS"))
  (shadow 'DepthTestMsg (find-package "FISH_MSGS")))
(unless (find-package "FISH_MSGS::DEPTHTESTMSG")
  (make-package "FISH_MSGS::DEPTHTESTMSG"))

(in-package "ROS")
;;//! \htmlinclude DepthTestMsg.msg.html


(defclass fish_msgs::DepthTestMsg
  :super ros::object
  :slots (_mode _value ))

(defmethod fish_msgs::DepthTestMsg
  (:init
   (&key
    ((:mode __mode) 0)
    ((:value __value) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _value (float __value))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int8 _mode
    1
    ;; float32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _mode
       (write-byte _mode s)
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get fish_msgs::DepthTestMsg :md5sum-) "392256b70b5280d59063ab9c2ddc1efa")
(setf (get fish_msgs::DepthTestMsg :datatype-) "fish_msgs/DepthTestMsg")
(setf (get fish_msgs::DepthTestMsg :definition-)
      "int8 mode
float32 value

")



(provide :fish_msgs/DepthTestMsg "392256b70b5280d59063ab9c2ddc1efa")


