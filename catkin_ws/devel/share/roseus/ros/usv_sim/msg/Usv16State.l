;; Auto-generated. Do not edit!


(when (boundp 'usv_sim::Usv16State)
  (if (not (find-package "USV_SIM"))
    (make-package "USV_SIM"))
  (shadow 'Usv16State (find-package "USV_SIM")))
(unless (find-package "USV_SIM::USV16STATE")
  (make-package "USV_SIM::USV16STATE"))

(in-package "ROS")
;;//! \htmlinclude Usv16State.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass usv_sim::Usv16State
  :super ros::object
  :slots (_t _pose _vel _acc ))

(defmethod usv_sim::Usv16State
  (:init
   (&key
    ((:t __t) 0.0)
    ((:pose __pose) (instance geometry_msgs::Pose2D :init))
    ((:vel __vel) (instance geometry_msgs::Twist :init))
    ((:acc __acc) (instance geometry_msgs::Twist :init))
    )
   (send-super :init)
   (setq _t (float __t))
   (setq _pose __pose)
   (setq _vel __vel)
   (setq _acc __acc)
   self)
  (:t
   (&optional __t)
   (if __t (setq _t __t)) _t)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:vel
   (&rest __vel)
   (if (keywordp (car __vel))
       (send* _vel __vel)
     (progn
       (if __vel (setq _vel (car __vel)))
       _vel)))
  (:acc
   (&rest __acc)
   (if (keywordp (car __acc))
       (send* _acc __acc)
     (progn
       (if __acc (setq _acc (car __acc)))
       _acc)))
  (:serialization-length
   ()
   (+
    ;; float64 _t
    8
    ;; geometry_msgs/Pose2D _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Twist _vel
    (send _vel :serialization-length)
    ;; geometry_msgs/Twist _acc
    (send _acc :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _t
       (sys::poke _t (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; geometry_msgs/Pose2D _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Twist _vel
       (send _vel :serialize s)
     ;; geometry_msgs/Twist _acc
       (send _acc :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _t
     (setq _t (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; geometry_msgs/Pose2D _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Twist _vel
     (send _vel :deserialize buf ptr-) (incf ptr- (send _vel :serialization-length))
   ;; geometry_msgs/Twist _acc
     (send _acc :deserialize buf ptr-) (incf ptr- (send _acc :serialization-length))
   ;;
   self)
  )

(setf (get usv_sim::Usv16State :md5sum-) "267dc2162d9371f64d61a71605f7ca88")
(setf (get usv_sim::Usv16State :datatype-) "usv_sim/Usv16State")
(setf (get usv_sim::Usv16State :definition-)
      "float64 t
geometry_msgs/Pose2D  pose
geometry_msgs/Twist  vel
geometry_msgs/Twist  acc

================================================================================
MSG: geometry_msgs/Pose2D
# This expresses a position and orientation on a 2D manifold.

float64 x
float64 y
float64 theta
================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :usv_sim/Usv16State "267dc2162d9371f64d61a71605f7ca88")


