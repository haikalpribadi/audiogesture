; Auto-generated. Do not edit!


(cl:in-package audiogesture-msg)


;//! \htmlinclude TrainerStatus.msg.html

(cl:defclass <TrainerStatus> (roslisp-msg-protocol:ros-message)
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

(cl:defclass TrainerStatus (<TrainerStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrainerStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrainerStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-msg:<TrainerStatus> is deprecated: use audiogesture-msg:TrainerStatus instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <TrainerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-msg:name-val is deprecated.  Use audiogesture-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <TrainerStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-msg:status-val is deprecated.  Use audiogesture-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrainerStatus>) ostream)
  "Serializes a message object of type '<TrainerStatus>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrainerStatus>) istream)
  "Deserializes a message object of type '<TrainerStatus>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrainerStatus>)))
  "Returns string type for a message object of type '<TrainerStatus>"
  "audiogesture/TrainerStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrainerStatus)))
  "Returns string type for a message object of type 'TrainerStatus"
  "audiogesture/TrainerStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrainerStatus>)))
  "Returns md5sum for a message object of type '<TrainerStatus>"
  "7e1e90d83383404e6cc132313b4f1148")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrainerStatus)))
  "Returns md5sum for a message object of type 'TrainerStatus"
  "7e1e90d83383404e6cc132313b4f1148")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrainerStatus>)))
  "Returns full string definition for message of type '<TrainerStatus>"
  (cl:format cl:nil "string name~%string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrainerStatus)))
  "Returns full string definition for message of type 'TrainerStatus"
  (cl:format cl:nil "string name~%string status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrainerStatus>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'status))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrainerStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'TrainerStatus
    (cl:cons ':name (name msg))
    (cl:cons ':status (status msg))
))
