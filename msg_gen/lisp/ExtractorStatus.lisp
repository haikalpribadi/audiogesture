; Auto-generated. Do not edit!


(cl:in-package audiogesture-msg)


;//! \htmlinclude ExtractorStatus.msg.html

(cl:defclass <ExtractorStatus> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (status
    :reader status
    :initarg :status
    :type cl:string
    :initform ""))
)

(cl:defclass ExtractorStatus (<ExtractorStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ExtractorStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ExtractorStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-msg:<ExtractorStatus> is deprecated: use audiogesture-msg:ExtractorStatus instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ExtractorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-msg:name-val is deprecated.  Use audiogesture-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <ExtractorStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-msg:status-val is deprecated.  Use audiogesture-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ExtractorStatus>) ostream)
  "Serializes a message object of type '<ExtractorStatus>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'status))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'status))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ExtractorStatus>) istream)
  "Deserializes a message object of type '<ExtractorStatus>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'status) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ExtractorStatus>)))
  "Returns string type for a message object of type '<ExtractorStatus>"
  "audiogesture/ExtractorStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ExtractorStatus)))
  "Returns string type for a message object of type 'ExtractorStatus"
  "audiogesture/ExtractorStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ExtractorStatus>)))
  "Returns md5sum for a message object of type '<ExtractorStatus>"
  "7e1e90d83383404e6cc132313b4f1148")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ExtractorStatus)))
  "Returns md5sum for a message object of type 'ExtractorStatus"
  "7e1e90d83383404e6cc132313b4f1148")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ExtractorStatus>)))
  "Returns full string definition for message of type '<ExtractorStatus>"
  (cl:format cl:nil "string name~%string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ExtractorStatus)))
  "Returns full string definition for message of type 'ExtractorStatus"
  (cl:format cl:nil "string name~%string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ExtractorStatus>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ExtractorStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'ExtractorStatus
    (cl:cons ':name (name msg))
    (cl:cons ':status (status msg))
))