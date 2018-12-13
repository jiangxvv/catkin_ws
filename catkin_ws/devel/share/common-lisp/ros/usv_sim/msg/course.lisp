; Auto-generated. Do not edit!


(cl:in-package usv_sim-msg)


;//! \htmlinclude course.msg.html

(cl:defclass <course> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (rate
    :reader rate
    :initarg :rate
    :type cl:float
    :initform 0.0)
   (acceleration
    :reader acceleration
    :initarg :acceleration
    :type cl:float
    :initform 0.0))
)

(cl:defclass course (<course>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <course>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'course)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usv_sim-msg:<course> is deprecated: use usv_sim-msg:course instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <course>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:angle-val is deprecated.  Use usv_sim-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'rate-val :lambda-list '(m))
(cl:defmethod rate-val ((m <course>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:rate-val is deprecated.  Use usv_sim-msg:rate instead.")
  (rate m))

(cl:ensure-generic-function 'acceleration-val :lambda-list '(m))
(cl:defmethod acceleration-val ((m <course>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:acceleration-val is deprecated.  Use usv_sim-msg:acceleration instead.")
  (acceleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <course>) ostream)
  "Serializes a message object of type '<course>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rate))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <course>) istream)
  "Deserializes a message object of type '<course>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rate) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'acceleration) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<course>)))
  "Returns string type for a message object of type '<course>"
  "usv_sim/course")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'course)))
  "Returns string type for a message object of type 'course"
  "usv_sim/course")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<course>)))
  "Returns md5sum for a message object of type '<course>"
  "ae838d50ec8e47c9ce31d29911fbe48c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'course)))
  "Returns md5sum for a message object of type 'course"
  "ae838d50ec8e47c9ce31d29911fbe48c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<course>)))
  "Returns full string definition for message of type '<course>"
  (cl:format cl:nil "float64 angle~%float64 rate~%float64 acceleration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'course)))
  "Returns full string definition for message of type 'course"
  (cl:format cl:nil "float64 angle~%float64 rate~%float64 acceleration~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <course>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <course>))
  "Converts a ROS message object to a list"
  (cl:list 'course
    (cl:cons ':angle (angle msg))
    (cl:cons ':rate (rate msg))
    (cl:cons ':acceleration (acceleration msg))
))
