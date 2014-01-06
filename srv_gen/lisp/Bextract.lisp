; Auto-generated. Do not edit!


(cl:in-package audiogesture-srv)


;//! \htmlinclude Bextract-request.msg.html

(cl:defclass <Bextract-request> (roslisp-msg-protocol:ros-message)
  ((args
    :reader args
    :initarg :args
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Bextract-request (<Bextract-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Bextract-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Bextract-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<Bextract-request> is deprecated: use audiogesture-srv:Bextract-request instead.")))

(cl:ensure-generic-function 'args-val :lambda-list '(m))
(cl:defmethod args-val ((m <Bextract-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-srv:args-val is deprecated.  Use audiogesture-srv:args instead.")
  (args m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bextract-request>) ostream)
  "Serializes a message object of type '<Bextract-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'args))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'args))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Bextract-request>) istream)
  "Deserializes a message object of type '<Bextract-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'args) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'args)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Bextract-request>)))
  "Returns string type for a service object of type '<Bextract-request>"
  "audiogesture/BextractRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bextract-request)))
  "Returns string type for a service object of type 'Bextract-request"
  "audiogesture/BextractRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Bextract-request>)))
  "Returns md5sum for a message object of type '<Bextract-request>"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bextract-request)))
  "Returns md5sum for a message object of type 'Bextract-request"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bextract-request>)))
  "Returns full string definition for message of type '<Bextract-request>"
  (cl:format cl:nil "string[] args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bextract-request)))
  "Returns full string definition for message of type 'Bextract-request"
  (cl:format cl:nil "string[] args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bextract-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'args) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bextract-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Bextract-request
    (cl:cons ':args (args msg))
))
;//! \htmlinclude Bextract-response.msg.html

(cl:defclass <Bextract-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Bextract-response (<Bextract-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Bextract-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Bextract-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<Bextract-response> is deprecated: use audiogesture-srv:Bextract-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bextract-response>) ostream)
  "Serializes a message object of type '<Bextract-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Bextract-response>) istream)
  "Deserializes a message object of type '<Bextract-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Bextract-response>)))
  "Returns string type for a service object of type '<Bextract-response>"
  "audiogesture/BextractResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bextract-response)))
  "Returns string type for a service object of type 'Bextract-response"
  "audiogesture/BextractResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Bextract-response>)))
  "Returns md5sum for a message object of type '<Bextract-response>"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bextract-response)))
  "Returns md5sum for a message object of type 'Bextract-response"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bextract-response>)))
  "Returns full string definition for message of type '<Bextract-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bextract-response)))
  "Returns full string definition for message of type 'Bextract-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bextract-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bextract-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Bextract-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Bextract)))
  'Bextract-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Bextract)))
  'Bextract-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bextract)))
  "Returns string type for a service object of type '<Bextract>"
  "audiogesture/Bextract")