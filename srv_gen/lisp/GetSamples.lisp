; Auto-generated. Do not edit!


(cl:in-package audiogesture-srv)


;//! \htmlinclude GetSamples-request.msg.html

(cl:defclass <GetSamples-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetSamples-request (<GetSamples-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSamples-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSamples-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<GetSamples-request> is deprecated: use audiogesture-srv:GetSamples-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSamples-request>) ostream)
  "Serializes a message object of type '<GetSamples-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSamples-request>) istream)
  "Deserializes a message object of type '<GetSamples-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSamples-request>)))
  "Returns string type for a service object of type '<GetSamples-request>"
  "audiogesture/GetSamplesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSamples-request)))
  "Returns string type for a service object of type 'GetSamples-request"
  "audiogesture/GetSamplesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSamples-request>)))
  "Returns md5sum for a message object of type '<GetSamples-request>"
  "a8d910579e9f95e2aed1c37cdce948ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSamples-request)))
  "Returns md5sum for a message object of type 'GetSamples-request"
  "a8d910579e9f95e2aed1c37cdce948ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSamples-request>)))
  "Returns full string definition for message of type '<GetSamples-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSamples-request)))
  "Returns full string definition for message of type 'GetSamples-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSamples-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSamples-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSamples-request
))
;//! \htmlinclude GetSamples-response.msg.html

(cl:defclass <GetSamples-response> (roslisp-msg-protocol:ros-message)
  ((samples
    :reader samples
    :initarg :samples
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element "")))
)

(cl:defclass GetSamples-response (<GetSamples-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetSamples-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetSamples-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-srv:<GetSamples-response> is deprecated: use audiogesture-srv:GetSamples-response instead.")))

(cl:ensure-generic-function 'samples-val :lambda-list '(m))
(cl:defmethod samples-val ((m <GetSamples-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-srv:samples-val is deprecated.  Use audiogesture-srv:samples instead.")
  (samples m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetSamples-response>) ostream)
  "Serializes a message object of type '<GetSamples-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'samples))))
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
   (cl:slot-value msg 'samples))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetSamples-response>) istream)
  "Deserializes a message object of type '<GetSamples-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'samples) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'samples)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetSamples-response>)))
  "Returns string type for a service object of type '<GetSamples-response>"
  "audiogesture/GetSamplesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSamples-response)))
  "Returns string type for a service object of type 'GetSamples-response"
  "audiogesture/GetSamplesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetSamples-response>)))
  "Returns md5sum for a message object of type '<GetSamples-response>"
  "a8d910579e9f95e2aed1c37cdce948ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetSamples-response)))
  "Returns md5sum for a message object of type 'GetSamples-response"
  "a8d910579e9f95e2aed1c37cdce948ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetSamples-response>)))
  "Returns full string definition for message of type '<GetSamples-response>"
  (cl:format cl:nil "string[] samples~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetSamples-response)))
  "Returns full string definition for message of type 'GetSamples-response"
  (cl:format cl:nil "string[] samples~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetSamples-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'samples) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetSamples-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetSamples-response
    (cl:cons ':samples (samples msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetSamples)))
  'GetSamples-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetSamples)))
  'GetSamples-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetSamples)))
  "Returns string type for a service object of type '<GetSamples>"
  "audiogesture/GetSamples")