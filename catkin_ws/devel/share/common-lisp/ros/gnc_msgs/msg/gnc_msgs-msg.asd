
(cl:in-package :asdf)

(defsystem "gnc_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "MotionStamped" :depends-on ("_package_MotionStamped"))
    (:file "_package_MotionStamped" :depends-on ("_package"))
    (:file "control" :depends-on ("_package_control"))
    (:file "_package_control" :depends-on ("_package"))
    (:file "course" :depends-on ("_package_course"))
    (:file "_package_course" :depends-on ("_package"))
  ))