; Auto-generated. Do not edit!


(cl:in-package audiogesture-msg)


;//! \htmlinclude PlayerCommand.msg.html

(cl:defclass <PlayerCommand> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (file
    :reader file
    :initarg :file
    :type cl:string
    :initform "")
   (command
    :reader command
    :initarg :command
    :type cl:string
    :initform ""))
)

(cl:defclass PlayerCommand (<PlayerCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PlayerCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PlayerCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name audiogesture-msg:<PlayerCommand> is deprecated: use audiogesture-msg:PlayerCommand instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <PlayerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-msg:name-val is deprecated.  Use audiogesture-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'file-val :lambda-list '(m))
(cl:defmethod file-val ((m <PlayerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-msg:file-val is deprecated.  Use audiogesture-msg:file instead.")
  (file m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <PlayerCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader audiogesture-msg:command-val is deprecated.  Use audiogesture-msg:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PlayerCommand>) ostream)
  "Serializes a message object of type '<PlayerCommand>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'file))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'file))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'command))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'command))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PlayerCommand>) istream)
  "Deserializes a message object of type '<PlayerCommand>"
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
      (cl:setf (cl:slot-value msg 'file) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'file) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'command) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PlayerCommand>)))
  "Returns string type for a message object of type '<PlayerCommand>"
  "audiogesture/PlayerCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PlayerCommand)))
  "Returns string type for a message object of type 'PlayerCommand"
  "audiogesture/PlayerCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PlayerCommand>)))
  "Returns md5sum for a message object of type '<PlayerCommand>"
  "8b4d0ce774dae1224469c9b8fe027ac5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PlayerCommand)))
  "Returns md5sum for a message object of type 'PlayerCommand"
  "8b4d0ce774dae1224469c9b8fe027ac5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PlayerCommand>)))
  "Returns full string definition for message of type '<PlayerCommand>"
  (cl:format cl:nil "string name~%string file~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PlayerCommand)))
  "Returns full string definition for message of type 'PlayerCommand"
  (cl:format cl:nil "string name~%string file~%string command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PlayerCommand>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'file))
     4 (cl:length (cl:slot-value msg 'command))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PlayerCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'PlayerCommand
    (cl:cons ':name (name msg))
    (cl:cons ':file (file msg))
    (cl:cons ':command (command msg))
))
