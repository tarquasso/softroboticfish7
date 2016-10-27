;; Auto-generated. Do not edit!


(when (boundp 'fish_msgs::MbedDataMsg)
  (if (not (find-package "FISH_MSGS"))
    (make-package "FISH_MSGS"))
  (shadow 'MbedDataMsg (find-package "FISH_MSGS")))
(unless (find-package "FISH_MSGS::MBEDDATAMSG")
  (make-package "FISH_MSGS::MBEDDATAMSG"))

(in-package "ROS")
;;//! \htmlinclude MbedDataMsg.msg.html


(defclass fish_msgs::MbedDataMsg
  :super ros::object
  :slots (_mode _thrust _dvalue _yaw _valve ))

(defmethod fish_msgs::MbedDataMsg
  (:init
   (&key
    ((:mode __mode) 0)
    ((:thrust __thrust) 0.0)
    ((:dvalue __dvalue) 0.0)
    ((:yaw __yaw) 0.0)
    ((:valve __valve) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _thrust (float __thrust))
   (setq _dvalue (float __dvalue))
   (setq _yaw (float __yaw))
   (setq _valve (float __valve))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:thrust
   (&optional __thrust)
   (if __thrust (setq _thrust __thrust)) _thrust)
  (:dvalue
   (&optional __dvalue)
   (if __dvalue (setq _dvalue __dvalue)) _dvalue)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:valve
   (&optional __valve)
   (if __valve (setq _valve __valve)) _valve)
  (:serialization-length
   ()
   (+
    ;; int8 _mode
    1
    ;; float32 _thrust
    4
    ;; float32 _dvalue
    4
    ;; float32 _yaw
    4
    ;; float32 _valve
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _mode
       (write-byte _mode s)
     ;; float32 _thrust
       (sys::poke _thrust (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _dvalue
       (sys::poke _dvalue (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _valve
       (sys::poke _valve (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; float32 _thrust
     (setq _thrust (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _dvalue
     (setq _dvalue (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _valve
     (setq _valve (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get fish_msgs::MbedDataMsg :md5sum-) "9305e87d9033eb2ef9f1c3927209a31b")
(setf (get fish_msgs::MbedDataMsg :datatype-) "fish_msgs/MbedDataMsg")
(setf (get fish_msgs::MbedDataMsg :definition-)
      "int8 mode
float32 thrust
float32 dvalue
float32 yaw
float32 valve

")



(provide :fish_msgs/MbedDataMsg "9305e87d9033eb2ef9f1c3927209a31b")


