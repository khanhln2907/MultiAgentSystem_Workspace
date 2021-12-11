;; Auto-generated. Do not edit!


(when (boundp 'tip::UnicycleInfoStruct)
  (if (not (find-package "TIP"))
    (make-package "TIP"))
  (shadow 'UnicycleInfoStruct (find-package "TIP")))
(unless (find-package "TIP::UNICYCLEINFOSTRUCT")
  (make-package "TIP::UNICYCLEINFOSTRUCT"))

(in-package "ROS")
;;//! \htmlinclude UnicycleInfoStruct.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tip::UnicycleInfoStruct
  :super ros::object
  :slots (_header _TransmitterID _AgentPosX _AgentPosY _AgentTheta _VirtualCenterX _VirtualCenterY _V_BLF ))

(defmethod tip::UnicycleInfoStruct
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:TransmitterID __TransmitterID) 0)
    ((:AgentPosX __AgentPosX) 0.0)
    ((:AgentPosY __AgentPosY) 0.0)
    ((:AgentTheta __AgentTheta) 0.0)
    ((:VirtualCenterX __VirtualCenterX) 0.0)
    ((:VirtualCenterY __VirtualCenterY) 0.0)
    ((:V_BLF __V_BLF) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _TransmitterID (round __TransmitterID))
   (setq _AgentPosX (float __AgentPosX))
   (setq _AgentPosY (float __AgentPosY))
   (setq _AgentTheta (float __AgentTheta))
   (setq _VirtualCenterX (float __VirtualCenterX))
   (setq _VirtualCenterY (float __VirtualCenterY))
   (setq _V_BLF (float __V_BLF))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:TransmitterID
   (&optional __TransmitterID)
   (if __TransmitterID (setq _TransmitterID __TransmitterID)) _TransmitterID)
  (:AgentPosX
   (&optional __AgentPosX)
   (if __AgentPosX (setq _AgentPosX __AgentPosX)) _AgentPosX)
  (:AgentPosY
   (&optional __AgentPosY)
   (if __AgentPosY (setq _AgentPosY __AgentPosY)) _AgentPosY)
  (:AgentTheta
   (&optional __AgentTheta)
   (if __AgentTheta (setq _AgentTheta __AgentTheta)) _AgentTheta)
  (:VirtualCenterX
   (&optional __VirtualCenterX)
   (if __VirtualCenterX (setq _VirtualCenterX __VirtualCenterX)) _VirtualCenterX)
  (:VirtualCenterY
   (&optional __VirtualCenterY)
   (if __VirtualCenterY (setq _VirtualCenterY __VirtualCenterY)) _VirtualCenterY)
  (:V_BLF
   (&optional __V_BLF)
   (if __V_BLF (setq _V_BLF __V_BLF)) _V_BLF)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _TransmitterID
    4
    ;; float32 _AgentPosX
    4
    ;; float32 _AgentPosY
    4
    ;; float32 _AgentTheta
    4
    ;; float32 _VirtualCenterX
    4
    ;; float32 _VirtualCenterY
    4
    ;; float32 _V_BLF
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _TransmitterID
       (write-long _TransmitterID s)
     ;; float32 _AgentPosX
       (sys::poke _AgentPosX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _AgentPosY
       (sys::poke _AgentPosY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _AgentTheta
       (sys::poke _AgentTheta (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _VirtualCenterX
       (sys::poke _VirtualCenterX (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _VirtualCenterY
       (sys::poke _VirtualCenterY (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _V_BLF
       (sys::poke _V_BLF (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _TransmitterID
     (setq _TransmitterID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _AgentPosX
     (setq _AgentPosX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _AgentPosY
     (setq _AgentPosY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _AgentTheta
     (setq _AgentTheta (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _VirtualCenterX
     (setq _VirtualCenterX (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _VirtualCenterY
     (setq _VirtualCenterY (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _V_BLF
     (setq _V_BLF (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tip::UnicycleInfoStruct :md5sum-) "5cea3926dc7b0a30646b151951d6b5f5")
(setf (get tip::UnicycleInfoStruct :datatype-) "tip/UnicycleInfoStruct")
(setf (get tip::UnicycleInfoStruct :definition-)
      "Header header
int32 TransmitterID
float32 AgentPosX
float32 AgentPosY
float32 AgentTheta
float32 VirtualCenterX
float32 VirtualCenterY
float32 V_BLF

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

")



(provide :tip/UnicycleInfoStruct "5cea3926dc7b0a30646b151951d6b5f5")


