;; Auto-generated. Do not edit!


(when (boundp 'gnc_msgs::MotionStamped)
  (if (not (find-package "GNC_MSGS"))
    (make-package "GNC_MSGS"))
  (shadow 'MotionStamped (find-package "GNC_MSGS")))
(unless (find-package "GNC_MSGS::MOTIONSTAMPED")
  (make-package "GNC_MSGS::MOTIONSTAMPED"))

(in-package "ROS")
;;//! \htmlinclude MotionStamped.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass gnc_msgs::MotionStamped
  :super ros::object
  :slots (_time _pose _vel _accel ))

(defmethod gnc_msgs::MotionStamped
  (:init
   (&key
    ((:time __time) 0.0)
    ((:pose __pose) (instance geometry_msgs::Vector3 :init))
    ((:vel __vel) (instance geometry_msgs::Vector3 :init))
    ((:accel __accel) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _time (float __time))
   (setq _pose __pose)
   (setq _vel __vel)
   (setq _accel __accel)
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
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
  (:accel
   (&rest __accel)
   (if (keywordp (car __accel))
       (send* _accel __accel)
     (progn
       (if __accel (setq _accel (car __accel)))
       _accel)))
  (:serialization-length
   ()
   (+
    ;; float32 _time
    4
    ;; geometry_msgs/Vector3 _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Vector3 _vel
    (send _vel :serialization-length)
    ;; geometry_msgs/Vector3 _accel
    (send _accel :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _time
       (sys::poke _time (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Vector3 _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Vector3 _vel
       (send _vel :serialize s)
     ;; geometry_msgs/Vector3 _accel
       (send _accel :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _time
     (setq _time (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Vector3 _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Vector3 _vel
     (send _vel :deserialize buf ptr-) (incf ptr- (send _vel :serialization-length))
   ;; geometry_msgs/Vector3 _accel
     (send _accel :deserialize buf ptr-) (incf ptr- (send _accel :serialization-length))
   ;;
   self)
  )

(setf (get gnc_msgs::MotionStamped :md5sum-) "c6633c63ac0d3fdc5b4d6fb7666e8181")
(setf (get gnc_msgs::MotionStamped :datatype-) "gnc_msgs/MotionStamped")
(setf (get gnc_msgs::MotionStamped :definition-)
      "float32 time
geometry_msgs/Vector3 pose
geometry_msgs/Vector3  vel
geometry_msgs/Vector3  accel
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



(provide :gnc_msgs/MotionStamped "c6633c63ac0d3fdc5b4d6fb7666e8181")


