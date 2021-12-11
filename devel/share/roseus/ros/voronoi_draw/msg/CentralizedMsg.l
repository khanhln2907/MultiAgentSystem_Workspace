;; Auto-generated. Do not edit!


(when (boundp 'voronoi_draw::CentralizedMsg)
  (if (not (find-package "VORONOI_DRAW"))
    (make-package "VORONOI_DRAW"))
  (shadow 'CentralizedMsg (find-package "VORONOI_DRAW")))
(unless (find-package "VORONOI_DRAW::CENTRALIZEDMSG")
  (make-package "VORONOI_DRAW::CENTRALIZEDMSG"))

(in-package "ROS")
;;//! \htmlinclude CentralizedMsg.msg.html


(defclass voronoi_draw::CentralizedMsg
  :super ros::object
  :slots (_valueArr ))

(defmethod voronoi_draw::CentralizedMsg
  (:init
   (&key
    ((:valueArr __valueArr) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _valueArr __valueArr)
   self)
  (:valueArr
   (&optional __valueArr)
   (if __valueArr (setq _valueArr __valueArr)) _valueArr)
  (:serialization-length
   ()
   (+
    ;; float32[] _valueArr
    (* 4    (length _valueArr)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _valueArr
     (write-long (length _valueArr) s)
     (dotimes (i (length _valueArr))
       (sys::poke (elt _valueArr i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _valueArr
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _valueArr (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _valueArr i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get voronoi_draw::CentralizedMsg :md5sum-) "06400f478916f41614d3caf5ce38a451")
(setf (get voronoi_draw::CentralizedMsg :datatype-) "voronoi_draw/CentralizedMsg")
(setf (get voronoi_draw::CentralizedMsg :definition-)
      "float32[] valueArr

")



(provide :voronoi_draw/CentralizedMsg "06400f478916f41614d3caf5ce38a451")


