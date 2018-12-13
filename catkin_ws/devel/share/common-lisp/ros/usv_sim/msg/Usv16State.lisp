; Auto-generated. Do not edit!


(cl:in-package usv_sim-msg)


;//! \htmlinclude Usv16State.msg.html

(cl:defclass <Usv16State> (roslisp-msg-protocol:ros-message)
  ((t
    :reader t
    :initarg :t
    :type cl:float
    :initform 0.0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Pose2D
    :initform (cl:make-instance 'geometry_msgs-msg:Pose2D))
   (vel
    :reader vel
    :initarg :vel
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist))
   (acc
    :reader acc
    :initarg :acc
    :type geometry_msgs-msg:Twist
    :initform (cl:make-instance 'geometry_msgs-msg:Twist)))
)

(cl:defclass Usv16State (<Usv16State>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Usv16State>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Usv16State)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usv_sim-msg:<Usv16State> is deprecated: use usv_sim-msg:Usv16State instead.")))

(cl:ensure-generic-function 't-val :lambda-list '(m))
(cl:defmethod t-val ((m <Usv16State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:t-val is deprecated.  Use usv_sim-msg:t instead.")
  (t m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <Usv16State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:pose-val is deprecated.  Use usv_sim-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <Usv16State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:vel-val is deprecated.  Use usv_sim-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'acc-val :lambda-list '(m))
(cl:defmethod acc-val ((m <Usv16State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:acc-val is deprecated.  Use usv_sim-msg:acc instead.")
  (acc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Usv16State>) ostream)
  "Serializes a message object of type '<Usv16State>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 't))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acc) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Usv16State>) istream)
  "Deserializes a message object of type '<Usv16State>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 't) (roslisp-utils:decode-double-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acc) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Usv16State>)))
  "Returns string type for a message object of type '<Usv16State>"
  "usv_sim/Usv16State")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Usv16State)))
  "Returns string type for a message object of type 'Usv16State"
  "usv_sim/Usv16State")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Usv16State>)))
  "Returns md5sum for a message object of type '<Usv16State>"
  "267dc2162d9371f64d61a71605f7ca88")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Usv16State)))
  "Returns md5sum for a message object of type 'Usv16State"
  "267dc2162d9371f64d61a71605f7ca88")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Usv16State>)))
  "Returns full string definition for message of type '<Usv16State>"
  (cl:format cl:nil "float64 t~%geometry_msgs/Pose2D  pose~%geometry_msgs/Twist  vel~%geometry_msgs/Twist  acc~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Usv16State)))
  "Returns full string definition for message of type 'Usv16State"
  (cl:format cl:nil "float64 t~%geometry_msgs/Pose2D  pose~%geometry_msgs/Twist  vel~%geometry_msgs/Twist  acc~%~%================================================================================~%MSG: geometry_msgs/Pose2D~%# This expresses a position and orientation on a 2D manifold.~%~%float64 x~%float64 y~%float64 theta~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Usv16State>))
  (cl:+ 0
     8
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acc))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Usv16State>))
  "Converts a ROS message object to a list"
  (cl:list 'Usv16State
    (cl:cons ':t (t msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':acc (acc msg))
))
