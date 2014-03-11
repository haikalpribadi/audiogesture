; Auto-generated. Do not edit!


(cl:in-package audiogesture-srv)


;//! \htmlinclude GetFile-request.msg.html

(cl:defclass <GetFile-request> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform ""))
)

(cl:defclass GetFile-request (<GetFile-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetFile-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetFile-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<GetFile-request> is deprecated: use audiogesture-srv:GetFile-request instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <GetFile-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-srv:name-val is deprecated.  Use audiogesture-srv:name instead.")
  (name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetFile-request>) ostream)
  "Serializes a message object of type '<GetFile-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetFile-request>) istream)
  "Deserializes a message object of type '<GetFile-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetFile-request>)))
  "Returns string type for a service object of type '<GetFile-request>"
  "audiogesture/GetFileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFile-request)))
  "Returns string type for a service object of type 'GetFile-request"
  "audiogesture/GetFileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetFile-request>)))
  "Returns md5sum for a message object of type '<GetFile-request>"
  "65741e102dfdf1ce689c0b8ed7c755c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetFile-request)))
  "Returns md5sum for a message object of type 'GetFile-request"
  "65741e102dfdf1ce689c0b8ed7c755c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetFile-request>)))
  "Returns full string definition for message of type '<GetFile-request>"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetFile-request)))
  "Returns full string definition for message of type 'GetFile-request"
  (cl:format cl:nil "string name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetFile-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetFile-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetFile-request
    (cl:cons ':name (name msg))
))
;//! \htmlinclude GetFile-response.msg.html

(cl:defclass <GetFile-response> (roslisp-msg-protocol:ros-message)
  ((file
    :reader file
    :initarg :file
    :type cl:string
    :initform ""))
)

(cl:defclass GetFile-response (<GetFile-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetFile-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetFile-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<GetFile-response> is deprecated: use audiogesture-srv:GetFile-response instead.")))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <GetFile-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-srv:file-val is deprecated.  Use audiogesture-srv:file instead.")
  (file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetFile-response>) ostream)
  "Serializes a message object of type '<GetFile-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetFile-response>) istream)
  "Deserializes a message object of type '<GetFile-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetFile-response>)))
  "Returns string type for a service object of type '<GetFile-response>"
  "audiogesture/GetFileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFile-response)))
  "Returns string type for a service object of type 'GetFile-response"
  "audiogesture/GetFileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetFile-response>)))
  "Returns md5sum for a message object of type '<GetFile-response>"
  "65741e102dfdf1ce689c0b8ed7c755c0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetFile-response)))
  "Returns md5sum for a message object of type 'GetFile-response"
  "65741e102dfdf1ce689c0b8ed7c755c0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetFile-response>)))
  "Returns full string definition for message of type '<GetFile-response>"
  (cl:format cl:nil "string file~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetFile-response)))
  "Returns full string definition for message of type 'GetFile-response"
  (cl:format cl:nil "string file~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetFile-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetFile-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetFile-response
    (cl:cons ':file (file msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetFile)))
  'GetFile-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetFile)))
  'GetFile-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetFile)))
  "Returns string type for a service object of type '<GetFile>"
  "audiogesture/GetFile")