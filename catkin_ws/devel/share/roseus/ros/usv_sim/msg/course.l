;; Auto-generated. Do not edit!


(when (boundp 'usv_sim::course)
  (if (not (find-package "USV_SIM"))
    (make-package "USV_SIM"))
  (shadow 'course (find-package "USV_SIM")))
(unless (find-package "USV_SIM::COURSE")
  (make-package "USV_SIM::COURSE"))

(in-package "ROS")
;;//! \htmlinclude course.msg.html


(defclass usv_sim::course
  :super ros::object
  :slots (_angle _rate _acceleration ))

(defmethod usv_sim::course
  (:init
   (&key
    ((:angle __angle) 0.0)
    ((:rate __rate) 0.0)
    ((:acceleration __acceleration) 0.0)
    )
   (send-super :init)
   (setq _angle (float __angle))
   (setq _rate (float __rate))
   (setq _acceleration (float __acceleration))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:rate
   (&optional __rate)
   (if __rate (setq _rate __rate)) _rate)
  (:acceleration
   (&optional __acceleration)
   (if __acceleration (setq _acceleration __acceleration)) _acceleration)
  (:serialization-length
   ()
   (+
    ;; float64 _angle
    8
    ;; float64 _rate
    8
    ;; float64 _acceleration
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _rate
       (sys::poke _rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _acceleration
       (sys::poke _acceleration (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _rate
     (setq _rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _acceleration
     (setq _acceleration (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get usv_sim::course :md5sum-) "ae838d50ec8e47c9ce31d29911fbe48c")
(setf (get usv_sim::course :datatype-) "usv_sim/course")
(setf (get usv_sim::course :definition-)
      "float64 angle
float64 rate
float64 acceleration

")



(provide :usv_sim/course "ae838d50ec8e47c9ce31d29911fbe48c")


