; Auto-generated. Do not edit!


(cl:in-package usv_sim-msg)


;//! \htmlinclude WaypointTrackingFeedback.msg.html

(cl:defclass <WaypointTrackingFeedback> (roslisp-msg-protocol:ros-message)
  ((progress
    :reader progress
    :initarg :progress
    :type cl:integer
    :initform 0))
)

(cl:defclass WaypointTrackingFeedback (<WaypointTrackingFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointTrackingFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointTrackingFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usv_sim-msg:<WaypointTrackingFeedback> is deprecated: use usv_sim-msg:WaypointTrackingFeedback instead.")))

(cl:ensure-generic-function 'progress-val :lambda-list '(m))
(cl:defmethod progress-val ((m <WaypointTrackingFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:progress-val is deprecated.  Use usv_sim-msg:progress instead.")
  (progress m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointTrackingFeedback>) ostream)
  "Serializes a message object of type '<WaypointTrackingFeedback>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'progress)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'progress)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'progress)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'progress)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointTrackingFeedback>) istream)
  "Deserializes a message object of type '<WaypointTrackingFeedback>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'progress)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'progress)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'progress)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'progress)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointTrackingFeedback>)))
  "Returns string type for a message object of type '<WaypointTrackingFeedback>"
  "usv_sim/WaypointTrackingFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointTrackingFeedback)))
  "Returns string type for a message object of type 'WaypointTrackingFeedback"
  "usv_sim/WaypointTrackingFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointTrackingFeedback>)))
  "Returns md5sum for a message object of type '<WaypointTrackingFeedback>"
  "6d745b453082f511ce5401257d5f4a34")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointTrackingFeedback)))
  "Returns md5sum for a message object of type 'WaypointTrackingFeedback"
  "6d745b453082f511ce5401257d5f4a34")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointTrackingFeedback>)))
  "Returns full string definition for message of type '<WaypointTrackingFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# feedback definition~%uint32 progress~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointTrackingFeedback)))
  "Returns full string definition for message of type 'WaypointTrackingFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# feedback definition~%uint32 progress~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointTrackingFeedback>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointTrackingFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointTrackingFeedback
    (cl:cons ':progress (progress msg))
))
