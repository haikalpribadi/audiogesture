; Auto-generated. Do not edit!


(cl:in-package audiogesture-srv)


;//! \htmlinclude Bextractor-request.msg.html

(cl:defclass <Bextractor-request> (roslisp-msg-protocol:ros-message)
  ((args
    :reader args
    :initarg :args
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass Bextractor-request (<Bextractor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Bextractor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Bextractor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<Bextractor-request> is deprecated: use audiogesture-srv:Bextractor-request instead.")))

(cl:ensure-generic-function 'args-val :lambda-list '(m))
(cl:defmethod args-val ((m <Bextractor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-srv:args-val is deprecated.  Use audiogesture-srv:args instead.")
  (args m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bextractor-request>) ostream)
  "Serializes a message object of type '<Bextractor-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Bextractor-request>) istream)
  "Deserializes a message object of type '<Bextractor-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Bextractor-request>)))
  "Returns string type for a service object of type '<Bextractor-request>"
  "audiogesture/BextractorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bextractor-request)))
  "Returns string type for a service object of type 'Bextractor-request"
  "audiogesture/BextractorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Bextractor-request>)))
  "Returns md5sum for a message object of type '<Bextractor-request>"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bextractor-request)))
  "Returns md5sum for a message object of type 'Bextractor-request"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bextractor-request>)))
  "Returns full string definition for message of type '<Bextractor-request>"
  (cl:format cl:nil "string[] args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bextractor-request)))
  "Returns full string definition for message of type 'Bextractor-request"
  (cl:format cl:nil "string[] args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bextractor-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'args) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bextractor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Bextractor-request
    (cl:cons ':args (args msg))
))
;//! \htmlinclude Bextractor-response.msg.html

(cl:defclass <Bextractor-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass Bextractor-response (<Bextractor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Bextractor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Bextractor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<Bextractor-response> is deprecated: use audiogesture-srv:Bextractor-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Bextractor-response>) ostream)
  "Serializes a message object of type '<Bextractor-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Bextractor-response>) istream)
  "Deserializes a message object of type '<Bextractor-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Bextractor-response>)))
  "Returns string type for a service object of type '<Bextractor-response>"
  "audiogesture/BextractorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bextractor-response)))
  "Returns string type for a service object of type 'Bextractor-response"
  "audiogesture/BextractorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Bextractor-response>)))
  "Returns md5sum for a message object of type '<Bextractor-response>"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Bextractor-response)))
  "Returns md5sum for a message object of type 'Bextractor-response"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Bextractor-response>)))
  "Returns full string definition for message of type '<Bextractor-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Bextractor-response)))
  "Returns full string definition for message of type 'Bextractor-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Bextractor-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Bextractor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Bextractor-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Bextractor)))
  'Bextractor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Bextractor)))
  'Bextractor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Bextractor)))
  "Returns string type for a service object of type '<Bextractor>"
  "audiogesture/Bextractor")