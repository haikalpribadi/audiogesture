; Auto-generated. Do not edit!


(cl:in-package audiogesture-srv)


;//! \htmlinclude GetSampleFile-request.msg.html

(cl:defclass <GetSampleFile-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass GetSampleFile-request (<GetSampleFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSampleFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSampleFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<GetSampleFile-request> is deprecated: use audiogesture-srv:GetSampleFile-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <GetSampleFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-srv:name-val is deprecated.  Use audiogesture-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSampleFile-request>) ostream)
  "Serializes a message object of type '<GetSampleFile-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSampleFile-request>) istream)
  "Deserializes a message object of type '<GetSampleFile-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSampleFile-request>)))
  "Returns string type for a service object of type '<GetSampleFile-request>"
  "audiogesture/GetSampleFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSampleFile-request)))
  "Returns string type for a service object of type 'GetSampleFile-request"
  "audiogesture/GetSampleFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSampleFile-request>)))
  "Returns md5sum for a message object of type '<GetSampleFile-request>"
  "65741e102dfdf1ce689c0b8ed7c755c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSampleFile-request)))
  "Returns md5sum for a message object of type 'GetSampleFile-request"
  "65741e102dfdf1ce689c0b8ed7c755c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSampleFile-request>)))
  "Returns full string definition for message of type '<GetSampleFile-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSampleFile-request)))
  "Returns full string definition for message of type 'GetSampleFile-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSampleFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSampleFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSampleFile-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude GetSampleFile-response.msg.html

(cl:defclass <GetSampleFile-response> (roslisp-msg-protocol:ros-message)
  ((file
    :reader file
    :initarg :file
    :type cl:string
    :initform ""))
)

(cl:defclass GetSampleFile-response (<GetSampleFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSampleFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSampleFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<GetSampleFile-response> is deprecated: use audiogesture-srv:GetSampleFile-response instead.")))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <GetSampleFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-srv:file-val is deprecated.  Use audiogesture-srv:file instead.")
  (file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSampleFile-response>) ostream)
  "Serializes a message object of type '<GetSampleFile-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSampleFile-response>) istream)
  "Deserializes a message object of type '<GetSampleFile-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSampleFile-response>)))
  "Returns string type for a service object of type '<GetSampleFile-response>"
  "audiogesture/GetSampleFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSampleFile-response)))
  "Returns string type for a service object of type 'GetSampleFile-response"
  "audiogesture/GetSampleFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSampleFile-response>)))
  "Returns md5sum for a message object of type '<GetSampleFile-response>"
  "65741e102dfdf1ce689c0b8ed7c755c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSampleFile-response)))
  "Returns md5sum for a message object of type 'GetSampleFile-response"
  "65741e102dfdf1ce689c0b8ed7c755c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSampleFile-response>)))
  "Returns full string definition for message of type '<GetSampleFile-response>"
  (cl:format cl:nil "string file~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSampleFile-response)))
  "Returns full string definition for message of type 'GetSampleFile-response"
  (cl:format cl:nil "string file~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSampleFile-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSampleFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSampleFile-response
    (cl:cons ':file (file msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSampleFile)))
  'GetSampleFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSampleFile)))
  'GetSampleFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSampleFile)))
  "Returns string type for a service object of type '<GetSampleFile>"
  "audiogesture/GetSampleFile")