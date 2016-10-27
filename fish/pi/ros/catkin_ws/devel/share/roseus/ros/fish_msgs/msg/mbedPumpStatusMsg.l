;; Auto-generated. Do not edit!


(when (boundp 'fish_msgs::mbedPumpStatusMsg)
  (if (not (find-package "FISH_MSGS"))
    (make-package "FISH_MSGS"))
  (shadow 'mbedPumpStatusMsg (find-package "FISH_MSGS")))
(unless (find-package "FISH_MSGS::MBEDPUMPSTATUSMSG")
  (make-package "FISH_MSGS::MBEDPUMPSTATUSMSG"))

(in-package "ROS")
;;//! \htmlinclude mbedPumpStatusMsg.msg.html


(defclass fish_msgs::mbedPumpStatusMsg
  :super ros::object
  :slots (_mode _yaw _freq ))

(defmethod fish_msgs::mbedPumpStatusMsg
  (:init
   (&key
    ((:mode __mode) 0)
    ((:yaw __yaw) 0.0)
    ((:freq __freq) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _yaw (float __yaw))
   (setq _freq (float __freq))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:freq
   (&optional __freq)
   (if __freq (setq _freq __freq)) _freq)
  (:serialization-length
   ()
   (+
    ;; int8 _mode
    1
    ;; float32 _yaw
    4
    ;; float32 _freq
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _mode
       (write-byte _mode s)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _freq
       (sys::poke _freq (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _freq
     (setq _freq (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get fish_msgs::mbedPumpStatusMsg :md5sum-) "d362e13e6bb848f15ea16da4058b765d")
(setf (get fish_msgs::mbedPumpStatusMsg :datatype-) "fish_msgs/mbedPumpStatusMsg")
(setf (get fish_msgs::mbedPumpStatusMsg :definition-)
      "int8 mode
float32 yaw
float32 freq

")



(provide :fish_msgs/mbedPumpStatusMsg "d362e13e6bb848f15ea16da4058b765d")


