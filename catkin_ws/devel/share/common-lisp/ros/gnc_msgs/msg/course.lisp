; Auto-generated. Do not edit!


(cl:in-package gnc_msgs-msg)


;//! \htmlinclude course.msg.html

(cl:defclass <course> (roslisp-msg-protocol:ros-message)
  ((pose
    :reader pose
    :initarg :pose
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (velocity
    :reader velocity
    :initarg :velocity
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass course (<course>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <course>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'course)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gnc_msgs-msg:<course> is deprecated: use gnc_msgs-msg:course instead.")))

(cl:ensure-generic-function 'pose-val :lambda-list '(m))
(cl:defmethod pose-val ((m <course>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gnc_msgs-msg:pose-val is deprecated.  Use gnc_msgs-msg:pose instead.")
  (pose m))

(cl:ensure-generic-function 'velocity-val :lambda-list '(m))
(cl:defmethod velocity-val ((m <course>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gnc_msgs-msg:velocity-val is deprecated.  Use gnc_msgs-msg:velocity instead.")
  (velocity m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <course>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gnc_msgs-msg:acceleration-val is deprecated.  Use gnc_msgs-msg:acceleration instead.")
  (acceleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <course>) ostream)
  "Serializes a message object of type '<course>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'acceleration) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <course>) istream)
  "Deserializes a message object of type '<course>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'acceleration) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<course>)))
  "Returns string type for a message object of type '<course>"
  "gnc_msgs/course")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'course)))
  "Returns string type for a message object of type 'course"
  "gnc_msgs/course")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<course>)))
  "Returns md5sum for a message object of type '<course>"
  "ee91ad718657acbe3311fc42870a477e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'course)))
  "Returns md5sum for a message object of type 'course"
  "ee91ad718657acbe3311fc42870a477e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<course>)))
  "Returns full string definition for message of type '<course>"
  (cl:format cl:nil "# course~%geometry_msgs/Vector3 pose~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'course)))
  "Returns full string definition for message of type 'course"
  (cl:format cl:nil "# course~%geometry_msgs/Vector3 pose~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <course>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'acceleration))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <course>))
  "Converts a ROS message object to a list"
  (cl:list 'course
    (cl:cons ':pose (pose msg))
    (cl:cons ':velocity (velocity msg))
    (cl:cons ':acceleration (acceleration msg))
))
