;; Auto-generated. Do not edit!


(when (boundp 'usv_sim::waypoint)
  (if (not (find-package "USV_SIM"))
    (make-package "USV_SIM"))
  (shadow 'waypoint (find-package "USV_SIM")))
(unless (find-package "USV_SIM::WAYPOINT")
  (make-package "USV_SIM::WAYPOINT"))
(unless (find-package "USV_SIM::WAYPOINTREQUEST")
  (make-package "USV_SIM::WAYPOINTREQUEST"))
(unless (find-package "USV_SIM::WAYPOINTRESPONSE")
  (make-package "USV_SIM::WAYPOINTRESPONSE"))

(in-package "ROS")





(defclass usv_sim::waypointRequest
  :super ros::object
  :slots (_x _y ))

(defmethod usv_sim::waypointRequest
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass usv_sim::waypointResponse
  :super ros::object
  :slots (_feedback ))

(defmethod usv_sim::waypointResponse
  (:init
   (&key
    ((:feedback __feedback) "")
    )
   (send-super :init)
   (setq _feedback (string __feedback))
   self)
  (:feedback
   (&optional __feedback)
   (if __feedback (setq _feedback __feedback)) _feedback)
  (:serialization-length
   ()
   (+
    ;; string _feedback
    4 (length _feedback)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _feedback
       (write-long (length _feedback) s) (princ _feedback s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _feedback
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _feedback (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass usv_sim::waypoint
  :super ros::object
  :slots ())

(setf (get usv_sim::waypoint :md5sum-) "2dfba887f0d6c81636f47bdde25045e3")
(setf (get usv_sim::waypoint :datatype-) "usv_sim/waypoint")
(setf (get usv_sim::waypoint :request) usv_sim::waypointRequest)
(setf (get usv_sim::waypoint :response) usv_sim::waypointResponse)

(defmethod usv_sim::waypointRequest
  (:response () (instance usv_sim::waypointResponse :init)))

(setf (get usv_sim::waypointRequest :md5sum-) "2dfba887f0d6c81636f47bdde25045e3")
(setf (get usv_sim::waypointRequest :datatype-) "usv_sim/waypointRequest")
(setf (get usv_sim::waypointRequest :definition-)
      "float64 x
float64 y
---
string feedback

")

(setf (get usv_sim::waypointResponse :md5sum-) "2dfba887f0d6c81636f47bdde25045e3")
(setf (get usv_sim::waypointResponse :datatype-) "usv_sim/waypointResponse")
(setf (get usv_sim::waypointResponse :definition-)
      "float64 x
float64 y
---
string feedback

")



(provide :usv_sim/waypoint "2dfba887f0d6c81636f47bdde25045e3")


