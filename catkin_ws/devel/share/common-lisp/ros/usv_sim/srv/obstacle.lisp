; Auto-generated. Do not edit!


(cl:in-package usv_sim-srv)


;//! \htmlinclude obstacle-request.msg.html

(cl:defclass <obstacle-request> (roslisp-msg-protocol:ros-message)
  ((x_coor
    :reader x_coor
    :initarg :x_coor
    :type cl:float
    :initform 0.0)
   (y_coor
    :reader y_coor
    :initarg :y_coor
    :type cl:float
    :initform 0.0)
   (radius
    :reader radius
    :initarg :radius
    :type cl:float
    :initform 0.0))
)

(cl:defclass obstacle-request (<obstacle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstacle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstacle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usv_sim-srv:<obstacle-request> is deprecated: use usv_sim-srv:obstacle-request instead.")))

(cl:ensure-generic-function 'x_coor-val :lambda-list '(m))
(cl:defmethod x_coor-val ((m <obstacle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-srv:x_coor-val is deprecated.  Use usv_sim-srv:x_coor instead.")
  (x_coor m))

(cl:ensure-generic-function 'y_coor-val :lambda-list '(m))
(cl:defmethod y_coor-val ((m <obstacle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-srv:y_coor-val is deprecated.  Use usv_sim-srv:y_coor instead.")
  (y_coor m))

(cl:ensure-generic-function 'radius-val :lambda-list '(m))
(cl:defmethod radius-val ((m <obstacle-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-srv:radius-val is deprecated.  Use usv_sim-srv:radius instead.")
  (radius m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstacle-request>) ostream)
  "Serializes a message object of type '<obstacle-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_coor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_coor))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'radius))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstacle-request>) istream)
  "Deserializes a message object of type '<obstacle-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_coor) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_coor) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'radius) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstacle-request>)))
  "Returns string type for a service object of type '<obstacle-request>"
  "usv_sim/obstacleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle-request)))
  "Returns string type for a service object of type 'obstacle-request"
  "usv_sim/obstacleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstacle-request>)))
  "Returns md5sum for a message object of type '<obstacle-request>"
  "11fe2b9b051393d66ed5d61bfe2c5ce7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstacle-request)))
  "Returns md5sum for a message object of type 'obstacle-request"
  "11fe2b9b051393d66ed5d61bfe2c5ce7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstacle-request>)))
  "Returns full string definition for message of type '<obstacle-request>"
  (cl:format cl:nil "float64 x_coor~%float64 y_coor~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstacle-request)))
  "Returns full string definition for message of type 'obstacle-request"
  (cl:format cl:nil "float64 x_coor~%float64 y_coor~%float64 radius~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstacle-request>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstacle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'obstacle-request
    (cl:cons ':x_coor (x_coor msg))
    (cl:cons ':y_coor (y_coor msg))
    (cl:cons ':radius (radius msg))
))
;//! \htmlinclude obstacle-response.msg.html

(cl:defclass <obstacle-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass obstacle-response (<obstacle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstacle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstacle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name usv_sim-srv:<obstacle-response> is deprecated: use usv_sim-srv:obstacle-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <obstacle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader usv_sim-srv:response-val is deprecated.  Use usv_sim-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstacle-response>) ostream)
  "Serializes a message object of type '<obstacle-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstacle-response>) istream)
  "Deserializes a message object of type '<obstacle-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstacle-response>)))
  "Returns string type for a service object of type '<obstacle-response>"
  "usv_sim/obstacleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle-response)))
  "Returns string type for a service object of type 'obstacle-response"
  "usv_sim/obstacleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstacle-response>)))
  "Returns md5sum for a message object of type '<obstacle-response>"
  "11fe2b9b051393d66ed5d61bfe2c5ce7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstacle-response)))
  "Returns md5sum for a message object of type 'obstacle-response"
  "11fe2b9b051393d66ed5d61bfe2c5ce7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstacle-response>)))
  "Returns full string definition for message of type '<obstacle-response>"
  (cl:format cl:nil "string response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstacle-response)))
  "Returns full string definition for message of type 'obstacle-response"
  (cl:format cl:nil "string response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstacle-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstacle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'obstacle-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'obstacle)))
  'obstacle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'obstacle)))
  'obstacle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle)))
  "Returns string type for a service object of type '<obstacle>"
  "usv_sim/obstacle")