; Auto-generated. Do not edit!


(cl:in-package audiogesture-srv)


;//! \htmlinclude MusicExtractor-request.msg.html

(cl:defclass <MusicExtractor-request> (roslisp-msg-protocol:ros-message)
  ((args
    :reader args
    :initarg :args
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass MusicExtractor-request (<MusicExtractor-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MusicExtractor-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MusicExtractor-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<MusicExtractor-request> is deprecated: use audiogesture-srv:MusicExtractor-request instead.")))

(cl:ensure-generic-function 'args-val :lambda-list '(m))
(cl:defmethod args-val ((m <MusicExtractor-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-srv:args-val is deprecated.  Use audiogesture-srv:args instead.")
  (args m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MusicExtractor-request>) ostream)
  "Serializes a message object of type '<MusicExtractor-request>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MusicExtractor-request>) istream)
  "Deserializes a message object of type '<MusicExtractor-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MusicExtractor-request>)))
  "Returns string type for a service object of type '<MusicExtractor-request>"
  "audiogesture/MusicExtractorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MusicExtractor-request)))
  "Returns string type for a service object of type 'MusicExtractor-request"
  "audiogesture/MusicExtractorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MusicExtractor-request>)))
  "Returns md5sum for a message object of type '<MusicExtractor-request>"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MusicExtractor-request)))
  "Returns md5sum for a message object of type 'MusicExtractor-request"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MusicExtractor-request>)))
  "Returns full string definition for message of type '<MusicExtractor-request>"
  (cl:format cl:nil "string[] args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MusicExtractor-request)))
  "Returns full string definition for message of type 'MusicExtractor-request"
  (cl:format cl:nil "string[] args~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MusicExtractor-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'args) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MusicExtractor-request>))
  "Converts a ROS message object to a list"
  (cl:list 'MusicExtractor-request
    (cl:cons ':args (args msg))
))
;//! \htmlinclude MusicExtractor-response.msg.html

(cl:defclass <MusicExtractor-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MusicExtractor-response (<MusicExtractor-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MusicExtractor-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MusicExtractor-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<MusicExtractor-response> is deprecated: use audiogesture-srv:MusicExtractor-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MusicExtractor-response>) ostream)
  "Serializes a message object of type '<MusicExtractor-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MusicExtractor-response>) istream)
  "Deserializes a message object of type '<MusicExtractor-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MusicExtractor-response>)))
  "Returns string type for a service object of type '<MusicExtractor-response>"
  "audiogesture/MusicExtractorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MusicExtractor-response)))
  "Returns string type for a service object of type 'MusicExtractor-response"
  "audiogesture/MusicExtractorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MusicExtractor-response>)))
  "Returns md5sum for a message object of type '<MusicExtractor-response>"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MusicExtractor-response)))
  "Returns md5sum for a message object of type 'MusicExtractor-response"
  "80df8955b7d65a9ccaa8145f1d8b2f01")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MusicExtractor-response>)))
  "Returns full string definition for message of type '<MusicExtractor-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MusicExtractor-response)))
  "Returns full string definition for message of type 'MusicExtractor-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MusicExtractor-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MusicExtractor-response>))
  "Converts a ROS message object to a list"
  (cl:list 'MusicExtractor-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'MusicExtractor)))
  'MusicExtractor-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'MusicExtractor)))
  'MusicExtractor-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MusicExtractor)))
  "Returns string type for a service object of type '<MusicExtractor>"
  "audiogesture/MusicExtractor")