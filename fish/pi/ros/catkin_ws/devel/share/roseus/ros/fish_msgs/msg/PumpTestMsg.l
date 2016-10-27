;; Auto-generated. Do not edit!


(when (boundp 'fish_msgs::PumpTestMsg)
  (if (not (find-package "FISH_MSGS"))
    (make-package "FISH_MSGS"))
  (shadow 'PumpTestMsg (find-package "FISH_MSGS")))
(unless (find-package "FISH_MSGS::PUMPTESTMSG")
  (make-package "FISH_MSGS::PUMPTESTMSG"))

(in-package "ROS")
;;//! \htmlinclude PumpTestMsg.msg.html


(defclass fish_msgs::PumpTestMsg
  :super ros::object
  :slots (_mode _freq _yaw ))

(defmethod fish_msgs::PumpTestMsg
  (:init
   (&key
    ((:mode __mode) 0)
    ((:freq __freq) 0.0)
    ((:yaw __yaw) 0.0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   (setq _freq (float __freq))
   (setq _yaw (float __yaw))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:freq
   (&optional __freq)
   (if __freq (setq _freq __freq)) _freq)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:serialization-length
   ()
   (+
    ;; int8 _mode
    1
    ;; float32 _freq
    4
    ;; float32 _yaw
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _mode
       (write-byte _mode s)
     ;; float32 _freq
       (sys::poke _freq (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _mode
     (setq _mode (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _mode 127) (setq _mode (- _mode 256)))
   ;; float32 _freq
     (setq _freq (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _yaw
     (setq _yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get fish_msgs::PumpTestMsg :md5sum-) "34fadd9771870a86de960e911d1d8c17")
(setf (get fish_msgs::PumpTestMsg :datatype-) "fish_msgs/PumpTestMsg")
(setf (get fish_msgs::PumpTestMsg :definition-)
      "int8 mode
float32 freq
float32 yaw

")



(provide :fish_msgs/PumpTestMsg "34fadd9771870a86de960e911d1d8c17")


