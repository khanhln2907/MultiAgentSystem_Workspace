; Auto-generated. Do not edit!


(cl:in-package voronoi_draw-msg)


;//! \htmlinclude CentralizedMsg.msg.html

(cl:defclass <CentralizedMsg> (roslisp-msg-protocol:ros-message)
  ((valueArr
    :reader valueArr
    :initarg :valueArr
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CentralizedMsg (<CentralizedMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CentralizedMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CentralizedMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name voronoi_draw-msg:<CentralizedMsg> is deprecated: use voronoi_draw-msg:CentralizedMsg instead.")))

(cl:ensure-generic-function 'valueArr-val :lambda-list '(m))
(cl:defmethod valueArr-val ((m <CentralizedMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voronoi_draw-msg:valueArr-val is deprecated.  Use voronoi_draw-msg:valueArr instead.")
  (valueArr m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CentralizedMsg>) ostream)
  "Serializes a message object of type '<CentralizedMsg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'valueArr))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'valueArr))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CentralizedMsg>) istream)
  "Deserializes a message object of type '<CentralizedMsg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'valueArr) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'valueArr)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CentralizedMsg>)))
  "Returns string type for a message object of type '<CentralizedMsg>"
  "voronoi_draw/CentralizedMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CentralizedMsg)))
  "Returns string type for a message object of type 'CentralizedMsg"
  "voronoi_draw/CentralizedMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CentralizedMsg>)))
  "Returns md5sum for a message object of type '<CentralizedMsg>"
  "06400f478916f41614d3caf5ce38a451")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CentralizedMsg)))
  "Returns md5sum for a message object of type 'CentralizedMsg"
  "06400f478916f41614d3caf5ce38a451")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CentralizedMsg>)))
  "Returns full string definition for message of type '<CentralizedMsg>"
  (cl:format cl:nil "float32[] valueArr~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CentralizedMsg)))
  "Returns full string definition for message of type 'CentralizedMsg"
  (cl:format cl:nil "float32[] valueArr~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CentralizedMsg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'valueArr) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CentralizedMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'CentralizedMsg
    (cl:cons ':valueArr (valueArr msg))
))
