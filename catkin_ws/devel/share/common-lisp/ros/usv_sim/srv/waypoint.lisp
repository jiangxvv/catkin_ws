; Auto-generated. Do not edit!


(cl:in-package usv_sim-srv)


;//! \htmlinclude waypoint-request.msg.html

(cl:defclass <waypoint-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0))
)

(cl:defclass waypoint-request (<waypoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <waypoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'waypoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usv_sim-srv:<waypoint-request> is deprecated: use usv_sim-srv:waypoint-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <waypoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-srv:x-val is deprecated.  Use usv_sim-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <waypoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-srv:y-val is deprecated.  Use usv_sim-srv:y instead.")
  (y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <waypoint-request>) ostream)
  "Serializes a message object of type '<waypoint-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <waypoint-request>) istream)
  "Deserializes a message object of type '<waypoint-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<waypoint-request>)))
  "Returns string type for a service object of type '<waypoint-request>"
  "usv_sim/waypointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'waypoint-request)))
  "Returns string type for a service object of type 'waypoint-request"
  "usv_sim/waypointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<waypoint-request>)))
  "Returns md5sum for a message object of type '<waypoint-request>"
  "2dfba887f0d6c81636f47bdde25045e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'waypoint-request)))
  "Returns md5sum for a message object of type 'waypoint-request"
  "2dfba887f0d6c81636f47bdde25045e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<waypoint-request>)))
  "Returns full string definition for message of type '<waypoint-request>"
  (cl:format cl:nil "float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'waypoint-request)))
  "Returns full string definition for message of type 'waypoint-request"
  (cl:format cl:nil "float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <waypoint-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <waypoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'waypoint-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
))
;//! \htmlinclude waypoint-response.msg.html

(cl:defclass <waypoint-response> (roslisp-msg-protocol:ros-message)
  ((feedback
    :reader feedback
    :initarg :feedback
    :type cl:string
    :initform ""))
)

(cl:defclass waypoint-response (<waypoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <waypoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'waypoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usv_sim-srv:<waypoint-response> is deprecated: use usv_sim-srv:waypoint-response instead.")))

(cl:ensure-generic-function 'feedback-val :lambda-list '(m))
(cl:defmethod feedback-val ((m <waypoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-srv:feedback-val is deprecated.  Use usv_sim-srv:feedback instead.")
  (feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <waypoint-response>) ostream)
  "Serializes a message object of type '<waypoint-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'feedback))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'feedback))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <waypoint-response>) istream)
  "Deserializes a message object of type '<waypoint-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'feedback) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'feedback) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<waypoint-response>)))
  "Returns string type for a service object of type '<waypoint-response>"
  "usv_sim/waypointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'waypoint-response)))
  "Returns string type for a service object of type 'waypoint-response"
  "usv_sim/waypointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<waypoint-response>)))
  "Returns md5sum for a message object of type '<waypoint-response>"
  "2dfba887f0d6c81636f47bdde25045e3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'waypoint-response)))
  "Returns md5sum for a message object of type 'waypoint-response"
  "2dfba887f0d6c81636f47bdde25045e3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<waypoint-response>)))
  "Returns full string definition for message of type '<waypoint-response>"
  (cl:format cl:nil "string feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'waypoint-response)))
  "Returns full string definition for message of type 'waypoint-response"
  (cl:format cl:nil "string feedback~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <waypoint-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <waypoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'waypoint-response
    (cl:cons ':feedback (feedback msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'waypoint)))
  'waypoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'waypoint)))
  'waypoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'waypoint)))
  "Returns string type for a service object of type '<waypoint>"
  "usv_sim/waypoint")