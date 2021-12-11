;; Auto-generated. Do not edit!


(when (boundp 'tip::UnicycleInfoMsg)
  (if (not (find-package "TIP"))
    (make-package "TIP"))
  (shadow 'UnicycleInfoMsg (find-package "TIP")))
(unless (find-package "TIP::UNICYCLEINFOMSG")
  (make-package "TIP::UNICYCLEINFOMSG"))

(in-package "ROS")
;;//! \htmlinclude UnicycleInfoMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tip::UnicycleInfoMsg
  :super ros::object
  :slots (_header _packet ))

(defmethod tip::UnicycleInfoMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:packet __packet) (instance tip::UnicycleInfoStruct :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _packet __packet)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:packet
   (&rest __packet)
   (if (keywordp (car __packet))
       (send* _packet __packet)
     (progn
       (if __packet (setq _packet (car __packet)))
       _packet)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; tip/UnicycleInfoStruct _packet
    (send _packet :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; tip/UnicycleInfoStruct _packet
       (send _packet :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; tip/UnicycleInfoStruct _packet
     (send _packet :deserialize buf ptr-) (incf ptr- (send _packet :serialization-length))
   ;;
   self)
  )

(setf (get tip::UnicycleInfoMsg :md5sum-) "17316b31155b675ddbba08720370f6af")
(setf (get tip::UnicycleInfoMsg :datatype-) "tip/UnicycleInfoMsg")
(setf (get tip::UnicycleInfoMsg :definition-)
      "Header header
UnicycleInfoStruct packet

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
# 0: no frame
# 1: global frame
string frame_id

================================================================================
MSG: tip/UnicycleInfoStruct
Header header
int32 TransmitterID
float32 AgentPosX
float32 AgentPosY
float32 AgentTheta
float32 VirtualCenterX
float32 VirtualCenterY
float32 V_BLF

")



(provide :tip/UnicycleInfoMsg "17316b31155b675ddbba08720370f6af")


