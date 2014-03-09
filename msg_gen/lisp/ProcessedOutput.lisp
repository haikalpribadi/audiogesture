; Auto-generated. Do not edit!


(cl:in-package audiogesture-msg)


;//! \htmlinclude ProcessedOutput.msg.html

(cl:defclass <ProcessedOutput> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (file
    :reader file
    :initarg :file
    :type cl:string
    :initform ""))
)

(cl:defclass ProcessedOutput (<ProcessedOutput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ProcessedOutput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ProcessedOutput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-msg:<ProcessedOutput> is deprecated: use audiogesture-msg:ProcessedOutput instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <ProcessedOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-msg:name-val is deprecated.  Use audiogesture-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <ProcessedOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-msg:type-val is deprecated.  Use audiogesture-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <ProcessedOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-msg:file-val is deprecated.  Use audiogesture-msg:file instead.")
  (file m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ProcessedOutput>) ostream)
  "Serializes a message object of type '<ProcessedOutput>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ProcessedOutput>) istream)
  "Deserializes a message object of type '<ProcessedOutput>"
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
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ProcessedOutput>)))
  "Returns string type for a message object of type '<ProcessedOutput>"
  "audiogesture/ProcessedOutput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ProcessedOutput)))
  "Returns string type for a message object of type 'ProcessedOutput"
  "audiogesture/ProcessedOutput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ProcessedOutput>)))
  "Returns md5sum for a message object of type '<ProcessedOutput>"
  "e1979c48e54b0e648564f0b60d474570")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ProcessedOutput)))
  "Returns md5sum for a message object of type 'ProcessedOutput"
  "e1979c48e54b0e648564f0b60d474570")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ProcessedOutput>)))
  "Returns full string definition for message of type '<ProcessedOutput>"
  (cl:format cl:nil "string name~%string type~%string file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ProcessedOutput)))
  "Returns full string definition for message of type 'ProcessedOutput"
  (cl:format cl:nil "string name~%string type~%string file~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ProcessedOutput>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:length (cl:slot-value msg 'file))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ProcessedOutput>))
  "Converts a ROS message object to a list"
  (cl:list 'ProcessedOutput
    (cl:cons ':name (name msg))
    (cl:cons ':type (type msg))
    (cl:cons ':file (file msg))
))
