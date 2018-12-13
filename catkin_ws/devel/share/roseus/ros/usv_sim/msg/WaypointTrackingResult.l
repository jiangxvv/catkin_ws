;; Auto-generated. Do not edit!


(when (boundp 'usv_sim::WaypointTrackingResult)
  (if (not (find-package "USV_SIM"))
    (make-package "USV_SIM"))
  (shadow 'WaypointTrackingResult (find-package "USV_SIM")))
(unless (find-package "USV_SIM::WAYPOINTTRACKINGRESULT")
  (make-package "USV_SIM::WAYPOINTTRACKINGRESULT"))

(in-package "ROS")
;;//! \htmlinclude WaypointTrackingResult.msg.html


(defclass usv_sim::WaypointTrackingResult
  :super ros::object
  :slots (_final_wpt ))

(defmethod usv_sim::WaypointTrackingResult
  (:init
   (&key
    ((:final_wpt __final_wpt) 0)
    )
   (send-super :init)
   (setq _final_wpt (round __final_wpt))
   self)
  (:final_wpt
   (&optional __final_wpt)
   (if __final_wpt (setq _final_wpt __final_wpt)) _final_wpt)
  (:serialization-length
   ()
   (+
    ;; uint32 _final_wpt
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _final_wpt
       (write-long _final_wpt s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _final_wpt
     (setq _final_wpt (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get usv_sim::WaypointTrackingResult :md5sum-) "7065bcd2aab1ea016ba8ab483931c6aa")
(setf (get usv_sim::WaypointTrackingResult :datatype-) "usv_sim/WaypointTrackingResult")
(setf (get usv_sim::WaypointTrackingResult :definition-)
      "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
# result definition
uint32 final_wpt

")



(provide :usv_sim/WaypointTrackingResult "7065bcd2aab1ea016ba8ab483931c6aa")

