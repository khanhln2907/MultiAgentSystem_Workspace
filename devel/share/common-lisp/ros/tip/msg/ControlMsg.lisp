; Auto-generated. Do not edit!


(cl:in-package tip-msg)


;//! \htmlinclude ControlMsg.msg.html

(cl:defclass <ControlMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (ID
    :reader ID
    :initarg :ID
    :type cl:integer
    :initform 0)
   (translation
    :reader translation
    :initarg :translation
    :type cl:float
    :initform 0.0)
   (rotation
    :reader rotation
    :initarg :rotation
    :type cl:float
    :initform 0.0))
)

(cl:defclass ControlMsg (<ControlMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ControlMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ControlMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tip-msg:<ControlMsg> is deprecated: use tip-msg:ControlMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ControlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:header-val is deprecated.  Use tip-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <ControlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:ID-val is deprecated.  Use tip-msg:ID instead.")
  (ID m))

(cl:ensure-generic-function 'translation-val :lambda-list '(m))
(cl:defmethod translation-val ((m <ControlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:translation-val is deprecated.  Use tip-msg:translation instead.")
  (translation m))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <ControlMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:rotation-val is deprecated.  Use tip-msg:rotation instead.")
  (rotation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ControlMsg>) ostream)
  "Serializes a message object of type '<ControlMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'translation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ControlMsg>) istream)
  "Deserializes a message object of type '<ControlMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'translation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotation) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ControlMsg>)))
  "Returns string type for a message object of type '<ControlMsg>"
  "tip/ControlMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ControlMsg)))
  "Returns string type for a message object of type 'ControlMsg"
  "tip/ControlMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ControlMsg>)))
  "Returns md5sum for a message object of type '<ControlMsg>"
  "a587840309303d749a13367bdb070c0c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ControlMsg)))
  "Returns md5sum for a message object of type 'ControlMsg"
  "a587840309303d749a13367bdb070c0c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ControlMsg>)))
  "Returns full string definition for message of type '<ControlMsg>"
  (cl:format cl:nil "Header header~%int32 ID~%float32 translation~%float32 rotation~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ControlMsg)))
  "Returns full string definition for message of type 'ControlMsg"
  (cl:format cl:nil "Header header~%int32 ID~%float32 translation~%float32 rotation~%~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ControlMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ControlMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'ControlMsg
    (cl:cons ':header (header msg))
    (cl:cons ':ID (ID msg))
    (cl:cons ':translation (translation msg))
    (cl:cons ':rotation (rotation msg))
))
