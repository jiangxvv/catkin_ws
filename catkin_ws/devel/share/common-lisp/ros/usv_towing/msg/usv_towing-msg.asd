
(cl:in-package :asdf)

(defsystem "usv_towing-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "VehicleState" :depends-on ("_package_VehicleState"))
    (:file "_package_VehicleState" :depends-on ("_package"))
    (:file "actuation" :depends-on ("_package_actuation"))
    (:file "_package_actuation" :depends-on ("_package"))
    (:file "control" :depends-on ("_package_control"))
    (:file "_package_control" :depends-on ("_package"))
    (:file "course" :depends-on ("_package_course"))
    (:file "_package_course" :depends-on ("_package"))
  ))