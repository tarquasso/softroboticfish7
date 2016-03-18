
(cl:in-package :asdf)

(defsystem "fish_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "joystick_in" :depends-on ("_package_joystick_in"))
    (:file "_package_joystick_in" :depends-on ("_package"))
  ))