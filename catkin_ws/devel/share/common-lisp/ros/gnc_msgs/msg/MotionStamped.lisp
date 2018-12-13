; Auto-generated. Do not edit!


(cl:in-package gnc_msgs-msg)


;//! \htmlinclude MotionStamped.msg.html

(cl:defclass <MotionStamped> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:float
    :initform 0.0)
   (pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (vel
    :reader vel
    :initarg :vel
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (accel
    :reader accel
    :initarg :accel
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass MotionStamped (<MotionStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotionStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotionStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gnc_msgs-msg:<MotionStamped> is deprecated: use gnc_msgs-msg:MotionStamped instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <MotionStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gnc_msgs-msg:time-val is deprecated.  Use gnc_msgs-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <MotionStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gnc_msgs-msg:pose-val is deprecated.  Use gnc_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <MotionStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gnc_msgs-msg:vel-val is deprecated.  Use gnc_msgs-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'accel-val :lambda-list '(m))
(cl:defmethod accel-val ((m <MotionStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gnc_msgs-msg:accel-val is deprecated.  Use gnc_msgs-msg:accel instead.")
  (accel m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotionStamped>) ostream)
  "Serializes a message object of type '<MotionStamped>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'vel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'accel) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotionStamped>) istream)
  "Deserializes a message object of type '<MotionStamped>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time) (roslisp-utils:decode-single-float-bits bits)))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'vel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'accel) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotionStamped>)))
  "Returns string type for a message object of type '<MotionStamped>"
  "gnc_msgs/MotionStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotionStamped)))
  "Returns string type for a message object of type 'MotionStamped"
  "gnc_msgs/MotionStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotionStamped>)))
  "Returns md5sum for a message object of type '<MotionStamped>"
  "c6633c63ac0d3fdc5b4d6fb7666e8181")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotionStamped)))
  "Returns md5sum for a message object of type 'MotionStamped"
  "c6633c63ac0d3fdc5b4d6fb7666e8181")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotionStamped>)))
  "Returns full string definition for message of type '<MotionStamped>"
  (cl:format cl:nil "float32 time~%geometry_msgs/Vector3 pose~%geometry_msgs/Vector3  vel~%geometry_msgs/Vector3  accel~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotionStamped)))
  "Returns full string definition for message of type 'MotionStamped"
  (cl:format cl:nil "float32 time~%geometry_msgs/Vector3 pose~%geometry_msgs/Vector3  vel~%geometry_msgs/Vector3  accel~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotionStamped>))
  (cl:+ 0
     4
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'vel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'accel))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotionStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'MotionStamped
    (cl:cons ':time (time msg))
    (cl:cons ':pose (pose msg))
    (cl:cons ':vel (vel msg))
    (cl:cons ':accel (accel msg))
))
