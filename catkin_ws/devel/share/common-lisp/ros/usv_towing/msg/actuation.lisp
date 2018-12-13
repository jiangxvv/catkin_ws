; Auto-generated. Do not edit!


(cl:in-package usv_towing-msg)


;//! \htmlinclude actuation.msg.html

(cl:defclass <actuation> (roslisp-msg-protocol:ros-message)
  ((tug1
    :reader tug1
    :initarg :tug1
    :type cl:float
    :initform 0.0)
   (tug2
    :reader tug2
    :initarg :tug2
    :type cl:float
    :initform 0.0)
   (tug3
    :reader tug3
    :initarg :tug3
    :type cl:float
    :initform 0.0)
   (tug4
    :reader tug4
    :initarg :tug4
    :type cl:float
    :initform 0.0))
)

(cl:defclass actuation (<actuation>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <actuation>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'actuation)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usv_towing-msg:<actuation> is deprecated: use usv_towing-msg:actuation instead.")))

(cl:ensure-generic-function 'tug1-val :lambda-list '(m))
(cl:defmethod tug1-val ((m <actuation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_towing-msg:tug1-val is deprecated.  Use usv_towing-msg:tug1 instead.")
  (tug1 m))

(cl:ensure-generic-function 'tug2-val :lambda-list '(m))
(cl:defmethod tug2-val ((m <actuation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_towing-msg:tug2-val is deprecated.  Use usv_towing-msg:tug2 instead.")
  (tug2 m))

(cl:ensure-generic-function 'tug3-val :lambda-list '(m))
(cl:defmethod tug3-val ((m <actuation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_towing-msg:tug3-val is deprecated.  Use usv_towing-msg:tug3 instead.")
  (tug3 m))

(cl:ensure-generic-function 'tug4-val :lambda-list '(m))
(cl:defmethod tug4-val ((m <actuation>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_towing-msg:tug4-val is deprecated.  Use usv_towing-msg:tug4 instead.")
  (tug4 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <actuation>) ostream)
  "Serializes a message object of type '<actuation>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tug1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tug2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tug3))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'tug4))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <actuation>) istream)
  "Deserializes a message object of type '<actuation>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tug1) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tug2) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tug3) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tug4) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<actuation>)))
  "Returns string type for a message object of type '<actuation>"
  "usv_towing/actuation")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'actuation)))
  "Returns string type for a message object of type 'actuation"
  "usv_towing/actuation")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<actuation>)))
  "Returns md5sum for a message object of type '<actuation>"
  "6d5d273aeba236b992836cce53bb09da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'actuation)))
  "Returns md5sum for a message object of type 'actuation"
  "6d5d273aeba236b992836cce53bb09da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<actuation>)))
  "Returns full string definition for message of type '<actuation>"
  (cl:format cl:nil "float64 tug1~%float64 tug2~%float64 tug3~%float64 tug4~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'actuation)))
  "Returns full string definition for message of type 'actuation"
  (cl:format cl:nil "float64 tug1~%float64 tug2~%float64 tug3~%float64 tug4~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <actuation>))
  (cl:+ 0
     8
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <actuation>))
  "Converts a ROS message object to a list"
  (cl:list 'actuation
    (cl:cons ':tug1 (tug1 msg))
    (cl:cons ':tug2 (tug2 msg))
    (cl:cons ':tug3 (tug3 msg))
    (cl:cons ':tug4 (tug4 msg))
))
