;; Auto-generated. Do not edit!


(when (boundp 'fish_msgs::joystick_in)
  (if (not (find-package "FISH_MSGS"))
    (make-package "FISH_MSGS"))
  (shadow 'joystick_in (find-package "FISH_MSGS")))
(unless (find-package "FISH_MSGS::JOYSTICK_IN")
  (make-package "FISH_MSGS::JOYSTICK_IN"))

(in-package "ROS")
;;//! \htmlinclude joystick_in.msg.html


(defclass fish_msgs::joystick_in
  :super ros::object
  :slots (_freq_ctrl _speed_ctrl _depth_ctrl _yaw_ctrl ))

(defmethod fish_msgs::joystick_in
  (:init
   (&key
    ((:freq_ctrl __freq_ctrl) 0)
    ((:speed_ctrl __speed_ctrl) 0)
    ((:depth_ctrl __depth_ctrl) 0)
    ((:yaw_ctrl __yaw_ctrl) 0)
    )
   (send-super :init)
   (setq _freq_ctrl (round __freq_ctrl))
   (setq _speed_ctrl (round __speed_ctrl))
   (setq _depth_ctrl (round __depth_ctrl))
   (setq _yaw_ctrl (round __yaw_ctrl))
   self)
  (:freq_ctrl
   (&optional __freq_ctrl)
   (if __freq_ctrl (setq _freq_ctrl __freq_ctrl)) _freq_ctrl)
  (:speed_ctrl
   (&optional __speed_ctrl)
   (if __speed_ctrl (setq _speed_ctrl __speed_ctrl)) _speed_ctrl)
  (:depth_ctrl
   (&optional __depth_ctrl)
   (if __depth_ctrl (setq _depth_ctrl __depth_ctrl)) _depth_ctrl)
  (:yaw_ctrl
   (&optional __yaw_ctrl)
   (if __yaw_ctrl (setq _yaw_ctrl __yaw_ctrl)) _yaw_ctrl)
  (:serialization-length
   ()
   (+
    ;; int8 _freq_ctrl
    1
    ;; int8 _speed_ctrl
    1
    ;; int8 _depth_ctrl
    1
    ;; int8 _yaw_ctrl
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _freq_ctrl
       (write-byte _freq_ctrl s)
     ;; int8 _speed_ctrl
       (write-byte _speed_ctrl s)
     ;; int8 _depth_ctrl
       (write-byte _depth_ctrl s)
     ;; int8 _yaw_ctrl
       (write-byte _yaw_ctrl s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _freq_ctrl
     (setq _freq_ctrl (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _freq_ctrl 127) (setq _freq_ctrl (- _freq_ctrl 256)))
   ;; int8 _speed_ctrl
     (setq _speed_ctrl (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _speed_ctrl 127) (setq _speed_ctrl (- _speed_ctrl 256)))
   ;; int8 _depth_ctrl
     (setq _depth_ctrl (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _depth_ctrl 127) (setq _depth_ctrl (- _depth_ctrl 256)))
   ;; int8 _yaw_ctrl
     (setq _yaw_ctrl (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _yaw_ctrl 127) (setq _yaw_ctrl (- _yaw_ctrl 256)))
   ;;
   self)
  )

(setf (get fish_msgs::joystick_in :md5sum-) "0c650c89727301b1e2298a1c19175b51")
(setf (get fish_msgs::joystick_in :datatype-) "fish_msgs/joystick_in")
(setf (get fish_msgs::joystick_in :definition-)
      "int8 freq_ctrl
int8 speed_ctrl
int8 depth_ctrl
int8 yaw_ctrl

")



(provide :fish_msgs/joystick_in "0c650c89727301b1e2298a1c19175b51")


