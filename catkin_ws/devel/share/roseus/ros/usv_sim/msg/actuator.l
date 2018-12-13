;; Auto-generated. Do not edit!


(when (boundp 'usv_sim::actuator)
  (if (not (find-package "USV_SIM"))
    (make-package "USV_SIM"))
  (shadow 'actuator (find-package "USV_SIM")))
(unless (find-package "USV_SIM::ACTUATOR")
  (make-package "USV_SIM::ACTUATOR"))

(in-package "ROS")
;;//! \htmlinclude actuator.msg.html


(defclass usv_sim::actuator
  :super ros::object
  :slots (_T_p _Alpha_p _T_s _Alpha_s ))

(defmethod usv_sim::actuator
  (:init
   (&key
    ((:T_p __T_p) 0.0)
    ((:Alpha_p __Alpha_p) 0.0)
    ((:T_s __T_s) 0.0)
    ((:Alpha_s __Alpha_s) 0.0)
    )
   (send-super :init)
   (setq _T_p (float __T_p))
   (setq _Alpha_p (float __Alpha_p))
   (setq _T_s (float __T_s))
   (setq _Alpha_s (float __Alpha_s))
   self)
  (:T_p
   (&optional __T_p)
   (if __T_p (setq _T_p __T_p)) _T_p)
  (:Alpha_p
   (&optional __Alpha_p)
   (if __Alpha_p (setq _Alpha_p __Alpha_p)) _Alpha_p)
  (:T_s
   (&optional __T_s)
   (if __T_s (setq _T_s __T_s)) _T_s)
  (:Alpha_s
   (&optional __Alpha_s)
   (if __Alpha_s (setq _Alpha_s __Alpha_s)) _Alpha_s)
  (:serialization-length
   ()
   (+
    ;; float64 _T_p
    8
    ;; float64 _Alpha_p
    8
    ;; float64 _T_s
    8
    ;; float64 _Alpha_s
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _T_p
       (sys::poke _T_p (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Alpha_p
       (sys::poke _Alpha_p (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _T_s
       (sys::poke _T_s (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _Alpha_s
       (sys::poke _Alpha_s (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _T_p
     (setq _T_p (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Alpha_p
     (setq _Alpha_p (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _T_s
     (setq _T_s (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _Alpha_s
     (setq _Alpha_s (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get usv_sim::actuator :md5sum-) "058476a8fb2069bcb447843b2fbad502")
(setf (get usv_sim::actuator :datatype-) "usv_sim/actuator")
(setf (get usv_sim::actuator :definition-)
      "float64 T_p
float64 Alpha_p
float64 T_s
float64 Alpha_s

")



(provide :usv_sim/actuator "058476a8fb2069bcb447843b2fbad502")


