;; Auto-generated. Do not edit!


(when (boundp 'usv_sim::obstacle)
  (if (not (find-package "USV_SIM"))
    (make-package "USV_SIM"))
  (shadow 'obstacle (find-package "USV_SIM")))
(unless (find-package "USV_SIM::OBSTACLE")
  (make-package "USV_SIM::OBSTACLE"))
(unless (find-package "USV_SIM::OBSTACLEREQUEST")
  (make-package "USV_SIM::OBSTACLEREQUEST"))
(unless (find-package "USV_SIM::OBSTACLERESPONSE")
  (make-package "USV_SIM::OBSTACLERESPONSE"))

(in-package "ROS")





(defclass usv_sim::obstacleRequest
  :super ros::object
  :slots (_x_coor _y_coor _radius ))

(defmethod usv_sim::obstacleRequest
  (:init
   (&key
    ((:x_coor __x_coor) 0.0)
    ((:y_coor __y_coor) 0.0)
    ((:radius __radius) 0.0)
    )
   (send-super :init)
   (setq _x_coor (float __x_coor))
   (setq _y_coor (float __y_coor))
   (setq _radius (float __radius))
   self)
  (:x_coor
   (&optional __x_coor)
   (if __x_coor (setq _x_coor __x_coor)) _x_coor)
  (:y_coor
   (&optional __y_coor)
   (if __y_coor (setq _y_coor __y_coor)) _y_coor)
  (:radius
   (&optional __radius)
   (if __radius (setq _radius __radius)) _radius)
  (:serialization-length
   ()
   (+
    ;; float64 _x_coor
    8
    ;; float64 _y_coor
    8
    ;; float64 _radius
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_coor
       (sys::poke _x_coor (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_coor
       (sys::poke _y_coor (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _radius
       (sys::poke _radius (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_coor
     (setq _x_coor (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_coor
     (setq _y_coor (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _radius
     (setq _radius (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass usv_sim::obstacleResponse
  :super ros::object
  :slots (_response ))

(defmethod usv_sim::obstacleResponse
  (:init
   (&key
    ((:response __response) "")
    )
   (send-super :init)
   (setq _response (string __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass usv_sim::obstacle
  :super ros::object
  :slots ())

(setf (get usv_sim::obstacle :md5sum-) "11fe2b9b051393d66ed5d61bfe2c5ce7")
(setf (get usv_sim::obstacle :datatype-) "usv_sim/obstacle")
(setf (get usv_sim::obstacle :request) usv_sim::obstacleRequest)
(setf (get usv_sim::obstacle :response) usv_sim::obstacleResponse)

(defmethod usv_sim::obstacleRequest
  (:response () (instance usv_sim::obstacleResponse :init)))

(setf (get usv_sim::obstacleRequest :md5sum-) "11fe2b9b051393d66ed5d61bfe2c5ce7")
(setf (get usv_sim::obstacleRequest :datatype-) "usv_sim/obstacleRequest")
(setf (get usv_sim::obstacleRequest :definition-)
      "float64 x_coor
float64 y_coor
float64 radius
---
string response
")

(setf (get usv_sim::obstacleResponse :md5sum-) "11fe2b9b051393d66ed5d61bfe2c5ce7")
(setf (get usv_sim::obstacleResponse :datatype-) "usv_sim/obstacleResponse")
(setf (get usv_sim::obstacleResponse :definition-)
      "float64 x_coor
float64 y_coor
float64 radius
---
string response
")



(provide :usv_sim/obstacle "11fe2b9b051393d66ed5d61bfe2c5ce7")


