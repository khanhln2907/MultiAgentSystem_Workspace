; Auto-generated. Do not edit!


(cl:in-package tip-msg)


;//! \htmlinclude UnicycleInfoStruct.msg.html

(cl:defclass <UnicycleInfoStruct> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (TransmitterID
    :reader TransmitterID
    :initarg :TransmitterID
    :type cl:integer
    :initform 0)
   (AgentPosX
    :reader AgentPosX
    :initarg :AgentPosX
    :type cl:float
    :initform 0.0)
   (AgentPosY
    :reader AgentPosY
    :initarg :AgentPosY
    :type cl:float
    :initform 0.0)
   (AgentTheta
    :reader AgentTheta
    :initarg :AgentTheta
    :type cl:float
    :initform 0.0)
   (VirtualCenterX
    :reader VirtualCenterX
    :initarg :VirtualCenterX
    :type cl:float
    :initform 0.0)
   (VirtualCenterY
    :reader VirtualCenterY
    :initarg :VirtualCenterY
    :type cl:float
    :initform 0.0)
   (V_BLF
    :reader V_BLF
    :initarg :V_BLF
    :type cl:float
    :initform 0.0))
)

(cl:defclass UnicycleInfoStruct (<UnicycleInfoStruct>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UnicycleInfoStruct>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UnicycleInfoStruct)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tip-msg:<UnicycleInfoStruct> is deprecated: use tip-msg:UnicycleInfoStruct instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <UnicycleInfoStruct>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:header-val is deprecated.  Use tip-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'TransmitterID-val :lambda-list '(m))
(cl:defmethod TransmitterID-val ((m <UnicycleInfoStruct>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:TransmitterID-val is deprecated.  Use tip-msg:TransmitterID instead.")
  (TransmitterID m))

(cl:ensure-generic-function 'AgentPosX-val :lambda-list '(m))
(cl:defmethod AgentPosX-val ((m <UnicycleInfoStruct>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:AgentPosX-val is deprecated.  Use tip-msg:AgentPosX instead.")
  (AgentPosX m))

(cl:ensure-generic-function 'AgentPosY-val :lambda-list '(m))
(cl:defmethod AgentPosY-val ((m <UnicycleInfoStruct>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:AgentPosY-val is deprecated.  Use tip-msg:AgentPosY instead.")
  (AgentPosY m))

(cl:ensure-generic-function 'AgentTheta-val :lambda-list '(m))
(cl:defmethod AgentTheta-val ((m <UnicycleInfoStruct>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:AgentTheta-val is deprecated.  Use tip-msg:AgentTheta instead.")
  (AgentTheta m))

(cl:ensure-generic-function 'VirtualCenterX-val :lambda-list '(m))
(cl:defmethod VirtualCenterX-val ((m <UnicycleInfoStruct>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:VirtualCenterX-val is deprecated.  Use tip-msg:VirtualCenterX instead.")
  (VirtualCenterX m))

(cl:ensure-generic-function 'VirtualCenterY-val :lambda-list '(m))
(cl:defmethod VirtualCenterY-val ((m <UnicycleInfoStruct>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:VirtualCenterY-val is deprecated.  Use tip-msg:VirtualCenterY instead.")
  (VirtualCenterY m))

(cl:ensure-generic-function 'V_BLF-val :lambda-list '(m))
(cl:defmethod V_BLF-val ((m <UnicycleInfoStruct>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:V_BLF-val is deprecated.  Use tip-msg:V_BLF instead.")
  (V_BLF m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UnicycleInfoStruct>) ostream)
  "Serializes a message object of type '<UnicycleInfoStruct>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'TransmitterID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'AgentPosX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'AgentPosY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'AgentTheta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'VirtualCenterX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'VirtualCenterY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'V_BLF))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UnicycleInfoStruct>) istream)
  "Deserializes a message object of type '<UnicycleInfoStruct>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'TransmitterID) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'AgentPosX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'AgentPosY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'AgentTheta) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VirtualCenterX) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'VirtualCenterY) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'V_BLF) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UnicycleInfoStruct>)))
  "Returns string type for a message object of type '<UnicycleInfoStruct>"
  "tip/UnicycleInfoStruct")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UnicycleInfoStruct)))
  "Returns string type for a message object of type 'UnicycleInfoStruct"
  "tip/UnicycleInfoStruct")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UnicycleInfoStruct>)))
  "Returns md5sum for a message object of type '<UnicycleInfoStruct>"
  "5cea3926dc7b0a30646b151951d6b5f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UnicycleInfoStruct)))
  "Returns md5sum for a message object of type 'UnicycleInfoStruct"
  "5cea3926dc7b0a30646b151951d6b5f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UnicycleInfoStruct>)))
  "Returns full string definition for message of type '<UnicycleInfoStruct>"
  (cl:format cl:nil "Header header~%int32 TransmitterID~%float32 AgentPosX~%float32 AgentPosY~%float32 AgentTheta~%float32 VirtualCenterX~%float32 VirtualCenterY~%float32 V_BLF~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UnicycleInfoStruct)))
  "Returns full string definition for message of type 'UnicycleInfoStruct"
  (cl:format cl:nil "Header header~%int32 TransmitterID~%float32 AgentPosX~%float32 AgentPosY~%float32 AgentTheta~%float32 VirtualCenterX~%float32 VirtualCenterY~%float32 V_BLF~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UnicycleInfoStruct>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UnicycleInfoStruct>))
  "Converts a ROS message object to a list"
  (cl:list 'UnicycleInfoStruct
    (cl:cons ':header (header msg))
    (cl:cons ':TransmitterID (TransmitterID msg))
    (cl:cons ':AgentPosX (AgentPosX msg))
    (cl:cons ':AgentPosY (AgentPosY msg))
    (cl:cons ':AgentTheta (AgentTheta msg))
    (cl:cons ':VirtualCenterX (VirtualCenterX msg))
    (cl:cons ':VirtualCenterY (VirtualCenterY msg))
    (cl:cons ':V_BLF (V_BLF msg))
))
