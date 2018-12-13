
(cl:in-package :asdf)

(defsystem "usv_sim-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "obstacle" :depends-on ("_package_obstacle"))
    (:file "_package_obstacle" :depends-on ("_package"))
    (:file "waypoint" :depends-on ("_package_waypoint"))
    (:file "_package_waypoint" :depends-on ("_package"))
  ))