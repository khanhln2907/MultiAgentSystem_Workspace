;; Auto-generated. Do not edit!


(when (boundp 'tip::ControlMsg)
  (if (not (find-package "TIP"))
    (make-package "TIP"))
  (shadow 'ControlMsg (find-package "TIP")))
(unless (find-package "TIP::CONTROLMSG")
  (make-package "TIP::CONTROLMSG"))

(in-package "ROS")
;;//! \htmlinclude ControlMsg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tip::ControlMsg
  :super ros::object
  :slots (_header _ID _translation _rotation ))

(defmethod tip::ControlMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:ID __ID) 0)
    ((:translation __translation) 0.0)
    ((:rotation __rotation) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _ID (round __ID))
   (setq _translation (float __translation))
   (setq _rotation (float __rotation))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:ID
   (&optional __ID)
   (if __ID (setq _ID __ID)) _ID)
  (:translation
   (&optional __translation)
   (if __translation (setq _translation __translation)) _translation)
  (:rotation
   (&optional __rotation)
   (if __rotation (setq _rotation __rotation)) _rotation)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32 _ID
    4
    ;; float32 _translation
    4
    ;; float32 _rotation
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32 _ID
       (write-long _ID s)
     ;; float32 _translation
       (sys::poke _translation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rotation
       (sys::poke _rotation (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32 _ID
     (setq _ID (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _translation
     (setq _translation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rotation
     (setq _rotation (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tip::ControlMsg :md5sum-) "a587840309303d749a13367bdb070c0c")
(setf (get tip::ControlMsg :datatype-) "tip/ControlMsg")
(setf (get tip::ControlMsg :definition-)
      "Header header
int32 ID
float32 translation
float32 rotation


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



(provide :tip/ControlMsg "a587840309303d749a13367bdb070c0c")


