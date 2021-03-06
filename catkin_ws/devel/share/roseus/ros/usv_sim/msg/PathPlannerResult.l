;; Auto-generated. Do not edit!


(when (boundp 'usv_sim::PathPlannerResult)
  (if (not (find-package "USV_SIM"))
    (make-package "USV_SIM"))
  (shadow 'PathPlannerResult (find-package "USV_SIM")))
(unless (find-package "USV_SIM::PATHPLANNERRESULT")
  (make-package "USV_SIM::PATHPLANNERRESULT"))

(in-package "ROS")
;;//! \htmlinclude PathPlannerResult.msg.html


(defclass usv_sim::PathPlannerResult
  :super ros::object
  :slots (_wpt_x _wpt_y _wpt_psi ))

(defmethod usv_sim::PathPlannerResult
  (:init
   (&key
    ((:wpt_x __wpt_x) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:wpt_y __wpt_y) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:wpt_psi __wpt_psi) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _wpt_x __wpt_x)
   (setq _wpt_y __wpt_y)
   (setq _wpt_psi __wpt_psi)
   self)
  (:wpt_x
   (&optional __wpt_x)
   (if __wpt_x (setq _wpt_x __wpt_x)) _wpt_x)
  (:wpt_y
   (&optional __wpt_y)
   (if __wpt_y (setq _wpt_y __wpt_y)) _wpt_y)
  (:wpt_psi
   (&optional __wpt_psi)
   (if __wpt_psi (setq _wpt_psi __wpt_psi)) _wpt_psi)
  (:serialization-length
   ()
   (+
    ;; float64[] _wpt_x
    (* 8    (length _wpt_x)) 4
    ;; float64[] _wpt_y
    (* 8    (length _wpt_y)) 4
    ;; float64[] _wpt_psi
    (* 8    (length _wpt_psi)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _wpt_x
     (write-long (length _wpt_x) s)
     (dotimes (i (length _wpt_x))
       (sys::poke (elt _wpt_x i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _wpt_y
     (write-long (length _wpt_y) s)
     (dotimes (i (length _wpt_y))
       (sys::poke (elt _wpt_y i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _wpt_psi
     (write-long (length _wpt_psi) s)
     (dotimes (i (length _wpt_psi))
       (sys::poke (elt _wpt_psi i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _wpt_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wpt_x (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _wpt_x i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _wpt_y
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wpt_y (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _wpt_y i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _wpt_psi
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _wpt_psi (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _wpt_psi i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get usv_sim::PathPlannerResult :md5sum-) "6a6cc66078fa7c117d6ab9ada7708794")
(setf (get usv_sim::PathPlannerResult :datatype-) "usv_sim/PathPlannerResult")
(setf (get usv_sim::PathPlannerResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# result definition
float64[] wpt_x
float64[] wpt_y
float64[] wpt_psi

")



(provide :usv_sim/PathPlannerResult "6a6cc66078fa7c117d6ab9ada7708794")


