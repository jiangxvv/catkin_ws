; Auto-generated. Do not edit!


(cl:in-package usv_sim-msg)


;//! \htmlinclude actuator.msg.html

(cl:defclass <actuator> (roslisp-msg-protocol:ros-message)
  ((T_p
    :reader T_p
    :initarg :T_p
    :type cl:float
    :initform 0.0)
   (Alpha_p
    :reader Alpha_p
    :initarg :Alpha_p
    :type cl:float
    :initform 0.0)
   (T_s
    :reader T_s
    :initarg :T_s
    :type cl:float
    :initform 0.0)
   (Alpha_s
    :reader Alpha_s
    :initarg :Alpha_s
    :type cl:float
    :initform 0.0))
)

(cl:defclass actuator (<actuator>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <actuator>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'actuator)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usv_sim-msg:<actuator> is deprecated: use usv_sim-msg:actuator instead.")))

(cl:ensure-generic-function 'T_p-val :lambda-list '(m))
(cl:defmethod T_p-val ((m <actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:T_p-val is deprecated.  Use usv_sim-msg:T_p instead.")
  (T_p m))

(cl:ensure-generic-function 'Alpha_p-val :lambda-list '(m))
(cl:defmethod Alpha_p-val ((m <actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:Alpha_p-val is deprecated.  Use usv_sim-msg:Alpha_p instead.")
  (Alpha_p m))

(cl:ensure-generic-function 'T_s-val :lambda-list '(m))
(cl:defmethod T_s-val ((m <actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:T_s-val is deprecated.  Use usv_sim-msg:T_s instead.")
  (T_s m))

(cl:ensure-generic-function 'Alpha_s-val :lambda-list '(m))
(cl:defmethod Alpha_s-val ((m <actuator>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-msg:Alpha_s-val is deprecated.  Use usv_sim-msg:Alpha_s instead.")
  (Alpha_s m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <actuator>) ostream)
  "Serializes a message object of type '<actuator>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'T_p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Alpha_p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'T_s))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Alpha_s))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <actuator>) istream)
  "Deserializes a message object of type '<actuator>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'T_p) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Alpha_p) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'T_s) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Alpha_s) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<actuator>)))
  "Returns string type for a message object of type '<actuator>"
  "usv_sim/actuator")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'actuator)))
  "Returns string type for a message object of type 'actuator"
  "usv_sim/actuator")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<actuator>)))
  "Returns md5sum for a message object of type '<actuator>"
  "058476a8fb2069bcb447843b2fbad502")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'actuator)))
  "Returns md5sum for a message object of type 'actuator"
  "058476a8fb2069bcb447843b2fbad502")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<actuator>)))
  "Returns full string definition for message of type '<actuator>"
  (cl:format cl:nil "float64 T_p~%float64 Alpha_p~%float64 T_s~%float64 Alpha_s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'actuator)))
  "Returns full string definition for message of type 'actuator"
  (cl:format cl:nil "float64 T_p~%float64 Alpha_p~%float64 T_s~%float64 Alpha_s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <actuator>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <actuator>))
  "Converts a ROS message object to a list"
  (cl:list 'actuator
    (cl:cons ':T_p (T_p msg))
    (cl:cons ':Alpha_p (Alpha_p msg))
    (cl:cons ':T_s (T_s msg))
    (cl:cons ':Alpha_s (Alpha_s msg))
))
