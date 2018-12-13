;; Auto-generated. Do not edit!


(when (boundp 'gnc_msgs::course)
  (if (not (find-package "GNC_MSGS"))
    (make-package "GNC_MSGS"))
  (shadow 'course (find-package "GNC_MSGS")))
(unless (find-package "GNC_MSGS::COURSE")
  (make-package "GNC_MSGS::COURSE"))

(in-package "ROS")
;;//! \htmlinclude course.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass gnc_msgs::course
  :super ros::object
  :slots (_pose _velocity _acceleration ))

(defmethod gnc_msgs::course
  (:init
   (&key
    ((:pose __pose) (instance geometry_msgs::Vector3 :init))
    ((:velocity __velocity) (instance geometry_msgs::Vector3 :init))
    ((:acceleration __acceleration) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _pose __pose)
   (setq _velocity __velocity)
   (setq _acceleration __acceleration)
   self)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:acceleration
   (&rest __acceleration)
   (if (keywordp (car __acceleration))
       (send* _acceleration __acceleration)
     (progn
       (if __acceleration (setq _acceleration (car __acceleration)))
       _acceleration)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _pose
    (send _pose :serialization-length)
    ;; geometry_msgs/Vector3 _velocity
    (send _velocity :serialization-length)
    ;; geometry_msgs/Vector3 _acceleration
    (send _acceleration :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _pose
       (send _pose :serialize s)
     ;; geometry_msgs/Vector3 _velocity
       (send _velocity :serialize s)
     ;; geometry_msgs/Vector3 _acceleration
       (send _acceleration :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; geometry_msgs/Vector3 _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; geometry_msgs/Vector3 _acceleration
     (send _acceleration :deserialize buf ptr-) (incf ptr- (send _acceleration :serialization-length))
   ;;
   self)
  )

(setf (get gnc_msgs::course :md5sum-) "ee91ad718657acbe3311fc42870a477e")
(setf (get gnc_msgs::course :datatype-) "gnc_msgs/course")
(setf (get gnc_msgs::course :definition-)
      "# course
geometry_msgs/Vector3 pose
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration

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



(provide :gnc_msgs/course "ee91ad718657acbe3311fc42870a477e")


