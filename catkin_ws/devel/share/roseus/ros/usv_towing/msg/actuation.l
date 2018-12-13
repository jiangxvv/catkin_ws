;; Auto-generated. Do not edit!


(when (boundp 'usv_towing::actuation)
  (if (not (find-package "USV_TOWING"))
    (make-package "USV_TOWING"))
  (shadow 'actuation (find-package "USV_TOWING")))
(unless (find-package "USV_TOWING::ACTUATION")
  (make-package "USV_TOWING::ACTUATION"))

(in-package "ROS")
;;//! \htmlinclude actuation.msg.html


(defclass usv_towing::actuation
  :super ros::object
  :slots (_tug1 _tug2 _tug3 _tug4 ))

(defmethod usv_towing::actuation
  (:init
   (&key
    ((:tug1 __tug1) 0.0)
    ((:tug2 __tug2) 0.0)
    ((:tug3 __tug3) 0.0)
    ((:tug4 __tug4) 0.0)
    )
   (send-super :init)
   (setq _tug1 (float __tug1))
   (setq _tug2 (float __tug2))
   (setq _tug3 (float __tug3))
   (setq _tug4 (float __tug4))
   self)
  (:tug1
   (&optional __tug1)
   (if __tug1 (setq _tug1 __tug1)) _tug1)
  (:tug2
   (&optional __tug2)
   (if __tug2 (setq _tug2 __tug2)) _tug2)
  (:tug3
   (&optional __tug3)
   (if __tug3 (setq _tug3 __tug3)) _tug3)
  (:tug4
   (&optional __tug4)
   (if __tug4 (setq _tug4 __tug4)) _tug4)
  (:serialization-length
   ()
   (+
    ;; float64 _tug1
    8
    ;; float64 _tug2
    8
    ;; float64 _tug3
    8
    ;; float64 _tug4
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _tug1
       (sys::poke _tug1 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tug2
       (sys::poke _tug2 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tug3
       (sys::poke _tug3 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _tug4
       (sys::poke _tug4 (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _tug1
     (setq _tug1 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tug2
     (setq _tug2 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tug3
     (setq _tug3 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _tug4
     (setq _tug4 (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get usv_towing::actuation :md5sum-) "6d5d273aeba236b992836cce53bb09da")
(setf (get usv_towing::actuation :datatype-) "usv_towing/actuation")
(setf (get usv_towing::actuation :definition-)
      "float64 tug1
float64 tug2
float64 tug3
float64 tug4
")



(provide :usv_towing/actuation "6d5d273aeba236b992836cce53bb09da")


