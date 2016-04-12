; Auto-generated. Do not edit!


(cl:in-package peg_sensor-srv)


;//! \htmlinclude String_cmd-request.msg.html

(cl:defclass <String_cmd-request> (roslisp-msg-protocol:ros-message)
  ((req
    :reader req
    :initarg :req
    :type cl:string
    :initform ""))
)

(cl:defclass String_cmd-request (<String_cmd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <String_cmd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'String_cmd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name peg_sensor-srv:<String_cmd-request> is deprecated: use peg_sensor-srv:String_cmd-request instead.")))

(cl:ensure-generic-function 'req-val :lambda-list '(m))
(cl:defmethod req-val ((m <String_cmd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader peg_sensor-srv:req-val is deprecated.  Use peg_sensor-srv:req instead.")
  (req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <String_cmd-request>) ostream)
  "Serializes a message object of type '<String_cmd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'req))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'req))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <String_cmd-request>) istream)
  "Deserializes a message object of type '<String_cmd-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'req) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'req) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<String_cmd-request>)))
  "Returns string type for a service object of type '<String_cmd-request>"
  "peg_sensor/String_cmdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'String_cmd-request)))
  "Returns string type for a service object of type 'String_cmd-request"
  "peg_sensor/String_cmdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<String_cmd-request>)))
  "Returns md5sum for a message object of type '<String_cmd-request>"
  "fd72814fc41c6bccdf8759d8dea09f77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'String_cmd-request)))
  "Returns md5sum for a message object of type 'String_cmd-request"
  "fd72814fc41c6bccdf8759d8dea09f77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<String_cmd-request>)))
  "Returns full string definition for message of type '<String_cmd-request>"
  (cl:format cl:nil "string req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'String_cmd-request)))
  "Returns full string definition for message of type 'String_cmd-request"
  (cl:format cl:nil "string req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <String_cmd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'req))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <String_cmd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'String_cmd-request
    (cl:cons ':req (req msg))
))
;//! \htmlinclude String_cmd-response.msg.html

(cl:defclass <String_cmd-response> (roslisp-msg-protocol:ros-message)
  ((res
    :reader res
    :initarg :res
    :type cl:string
    :initform ""))
)

(cl:defclass String_cmd-response (<String_cmd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <String_cmd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'String_cmd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name peg_sensor-srv:<String_cmd-response> is deprecated: use peg_sensor-srv:String_cmd-response instead.")))

(cl:ensure-generic-function 'res-val :lambda-list '(m))
(cl:defmethod res-val ((m <String_cmd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader peg_sensor-srv:res-val is deprecated.  Use peg_sensor-srv:res instead.")
  (res m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <String_cmd-response>) ostream)
  "Serializes a message object of type '<String_cmd-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'res))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'res))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <String_cmd-response>) istream)
  "Deserializes a message object of type '<String_cmd-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'res) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'res) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<String_cmd-response>)))
  "Returns string type for a service object of type '<String_cmd-response>"
  "peg_sensor/String_cmdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'String_cmd-response)))
  "Returns string type for a service object of type 'String_cmd-response"
  "peg_sensor/String_cmdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<String_cmd-response>)))
  "Returns md5sum for a message object of type '<String_cmd-response>"
  "fd72814fc41c6bccdf8759d8dea09f77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'String_cmd-response)))
  "Returns md5sum for a message object of type 'String_cmd-response"
  "fd72814fc41c6bccdf8759d8dea09f77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<String_cmd-response>)))
  "Returns full string definition for message of type '<String_cmd-response>"
  (cl:format cl:nil "string res~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'String_cmd-response)))
  "Returns full string definition for message of type 'String_cmd-response"
  (cl:format cl:nil "string res~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <String_cmd-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'res))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <String_cmd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'String_cmd-response
    (cl:cons ':res (res msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'String_cmd)))
  'String_cmd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'String_cmd)))
  'String_cmd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'String_cmd)))
  "Returns string type for a service object of type '<String_cmd>"
  "peg_sensor/String_cmd")