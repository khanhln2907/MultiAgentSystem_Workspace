; Auto-generated. Do not edit!


(cl:in-package tip-msg)


;//! \htmlinclude UnicycleInfoMsg.msg.html

(cl:defclass <UnicycleInfoMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (packet
    :reader packet
    :initarg :packet
    :type tip-msg:UnicycleInfoStruct
    :initform (cl:make-instance 'tip-msg:UnicycleInfoStruct)))
)

(cl:defclass UnicycleInfoMsg (<UnicycleInfoMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UnicycleInfoMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UnicycleInfoMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tip-msg:<UnicycleInfoMsg> is deprecated: use tip-msg:UnicycleInfoMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <UnicycleInfoMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:header-val is deprecated.  Use tip-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'packet-val :lambda-list '(m))
(cl:defmethod packet-val ((m <UnicycleInfoMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tip-msg:packet-val is deprecated.  Use tip-msg:packet instead.")
  (packet m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UnicycleInfoMsg>) ostream)
  "Serializes a message object of type '<UnicycleInfoMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'packet) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UnicycleInfoMsg>) istream)
  "Deserializes a message object of type '<UnicycleInfoMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'packet) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UnicycleInfoMsg>)))
  "Returns string type for a message object of type '<UnicycleInfoMsg>"
  "tip/UnicycleInfoMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UnicycleInfoMsg)))
  "Returns string type for a message object of type 'UnicycleInfoMsg"
  "tip/UnicycleInfoMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UnicycleInfoMsg>)))
  "Returns md5sum for a message object of type '<UnicycleInfoMsg>"
  "17316b31155b675ddbba08720370f6af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UnicycleInfoMsg)))
  "Returns md5sum for a message object of type 'UnicycleInfoMsg"
  "17316b31155b675ddbba08720370f6af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UnicycleInfoMsg>)))
  "Returns full string definition for message of type '<UnicycleInfoMsg>"
  (cl:format cl:nil "Header header~%UnicycleInfoStruct packet~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: tip/UnicycleInfoStruct~%Header header~%int32 TransmitterID~%float32 AgentPosX~%float32 AgentPosY~%float32 AgentTheta~%float32 VirtualCenterX~%float32 VirtualCenterY~%float32 V_BLF~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UnicycleInfoMsg)))
  "Returns full string definition for message of type 'UnicycleInfoMsg"
  (cl:format cl:nil "Header header~%UnicycleInfoStruct packet~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: tip/UnicycleInfoStruct~%Header header~%int32 TransmitterID~%float32 AgentPosX~%float32 AgentPosY~%float32 AgentTheta~%float32 VirtualCenterX~%float32 VirtualCenterY~%float32 V_BLF~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UnicycleInfoMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'packet))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UnicycleInfoMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'UnicycleInfoMsg
    (cl:cons ':header (header msg))
    (cl:cons ':packet (packet msg))
))
