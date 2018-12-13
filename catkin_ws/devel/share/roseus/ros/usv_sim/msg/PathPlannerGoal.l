;; Auto-generated. Do not edit!


(when (boundp 'usv_sim::PathPlannerGoal)
  (if (not (find-package "USV_SIM"))
    (make-package "USV_SIM"))
  (shadow 'PathPlannerGoal (find-package "USV_SIM")))
(unless (find-package "USV_SIM::PATHPLANNERGOAL")
  (make-package "USV_SIM::PATHPLANNERGOAL"))

(in-package "ROS")
;;//! \htmlinclude PathPlannerGoal.msg.html


(defclass usv_sim::PathPlannerGoal
  :super ros::object
  :slots (_start_pose _start_vel _end_pose ))

(defmethod usv_sim::PathPlannerGoal
  (:init
   (&key
    ((:start_pose __start_pose) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:start_vel __start_vel) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:end_pose __end_pose) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _start_pose __start_pose)
   (setq _start_vel __start_vel)
   (setq _end_pose __end_pose)
   self)
  (:start_pose
   (&optional __start_pose)
   (if __start_pose (setq _start_pose __start_pose)) _start_pose)
  (:start_vel
   (&optional __start_vel)
   (if __start_vel (setq _start_vel __start_vel)) _start_vel)
  (:end_pose
   (&optional __end_pose)
   (if __end_pose (setq _end_pose __end_pose)) _end_pose)
  (:serialization-length
   ()
   (+
    ;; float64[] _start_pose
    (* 8    (length _start_pose)) 4
    ;; float64[] _start_vel
    (* 8    (length _start_vel)) 4
    ;; float64[] _end_pose
    (* 8    (length _end_pose)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[] _start_pose
     (write-long (length _start_pose) s)
     (dotimes (i (length _start_pose))
       (sys::poke (elt _start_pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _start_vel
     (write-long (length _start_vel) s)
     (dotimes (i (length _start_vel))
       (sys::poke (elt _start_vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _end_pose
     (write-long (length _end_pose) s)
     (dotimes (i (length _end_pose))
       (sys::poke (elt _end_pose i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[] _start_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _start_pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _start_pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _start_vel
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _start_vel (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _start_vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _end_pose
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _end_pose (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _end_pose i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get usv_sim::PathPlannerGoal :md5sum-) "87cf7bdada84db78bc034034d07024b6")
(setf (get usv_sim::PathPlannerGoal :datatype-) "usv_sim/PathPlannerGoal")
(setf (get usv_sim::PathPlannerGoal :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# goal definition
float64[] start_pose
float64[] start_vel
float64[] end_pose

")



(provide :usv_sim/PathPlannerGoal "87cf7bdada84db78bc034034d07024b6")

