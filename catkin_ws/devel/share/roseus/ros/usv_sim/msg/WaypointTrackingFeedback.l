;; Auto-generated. Do not edit!


(when (boundp 'usv_sim::WaypointTrackingFeedback)
  (if (not (find-package "USV_SIM"))
    (make-package "USV_SIM"))
  (shadow 'WaypointTrackingFeedback (find-package "USV_SIM")))
(unless (find-package "USV_SIM::WAYPOINTTRACKINGFEEDBACK")
  (make-package "USV_SIM::WAYPOINTTRACKINGFEEDBACK"))

(in-package "ROS")
;;//! \htmlinclude WaypointTrackingFeedback.msg.html


(defclass usv_sim::WaypointTrackingFeedback
  :super ros::object
  :slots (_progress ))

(defmethod usv_sim::WaypointTrackingFeedback
  (:init
   (&key
    ((:progress __progress) 0)
    )
   (send-super :init)
   (setq _progress (round __progress))
   self)
  (:progress
   (&optional __progress)
   (if __progress (setq _progress __progress)) _progress)
  (:serialization-length
   ()
   (+
    ;; uint32 _progress
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _progress
       (write-long _progress s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _progress
     (setq _progress (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get usv_sim::WaypointTrackingFeedback :md5sum-) "6d745b453082f511ce5401257d5f4a34")
(setf (get usv_sim::WaypointTrackingFeedback :datatype-) "usv_sim/WaypointTrackingFeedback")
(setf (get usv_sim::WaypointTrackingFeedback :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# feedback definition
uint32 progress


")



(provide :usv_sim/WaypointTrackingFeedback "6d745b453082f511ce5401257d5f4a34")


