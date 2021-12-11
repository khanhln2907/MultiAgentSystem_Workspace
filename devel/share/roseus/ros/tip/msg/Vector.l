;; Auto-generated. Do not edit!


(when (boundp 'tip::Vector)
  (if (not (find-package "TIP"))
    (make-package "TIP"))
  (shadow 'Vector (find-package "TIP")))
(unless (find-package "TIP::VECTOR")
  (make-package "TIP::VECTOR"))

(in-package "ROS")
;;//! \htmlinclude Vector.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass tip::Vector
  :super ros::object
  :slots (_header _x1 _y1 _x2 _y2 _x3 _y3 _x4 _y4 _x5 _y5 _x6 _y6 ))

(defmethod tip::Vector
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:x1 __x1) 0)
    ((:y1 __y1) 0)
    ((:x2 __x2) 0)
    ((:y2 __y2) 0)
    ((:x3 __x3) 0)
    ((:y3 __y3) 0)
    ((:x4 __x4) 0)
    ((:y4 __y4) 0)
    ((:x5 __x5) 0)
    ((:y5 __y5) 0)
    ((:x6 __x6) 0)
    ((:y6 __y6) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _x1 (round __x1))
   (setq _y1 (round __y1))
   (setq _x2 (round __x2))
   (setq _y2 (round __y2))
   (setq _x3 (round __x3))
   (setq _y3 (round __y3))
   (setq _x4 (round __x4))
   (setq _y4 (round __y4))
   (setq _x5 (round __x5))
   (setq _y5 (round __y5))
   (setq _x6 (round __x6))
   (setq _y6 (round __y6))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:x1
   (&optional __x1)
   (if __x1 (setq _x1 __x1)) _x1)
  (:y1
   (&optional __y1)
   (if __y1 (setq _y1 __y1)) _y1)
  (:x2
   (&optional __x2)
   (if __x2 (setq _x2 __x2)) _x2)
  (:y2
   (&optional __y2)
   (if __y2 (setq _y2 __y2)) _y2)
  (:x3
   (&optional __x3)
   (if __x3 (setq _x3 __x3)) _x3)
  (:y3
   (&optional __y3)
   (if __y3 (setq _y3 __y3)) _y3)
  (:x4
   (&optional __x4)
   (if __x4 (setq _x4 __x4)) _x4)
  (:y4
   (&optional __y4)
   (if __y4 (setq _y4 __y4)) _y4)
  (:x5
   (&optional __x5)
   (if __x5 (setq _x5 __x5)) _x5)
  (:y5
   (&optional __y5)
   (if __y5 (setq _y5 __y5)) _y5)
  (:x6
   (&optional __x6)
   (if __x6 (setq _x6 __x6)) _x6)
  (:y6
   (&optional __y6)
   (if __y6 (setq _y6 __y6)) _y6)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int64 _x1
    8
    ;; int64 _y1
    8
    ;; int64 _x2
    8
    ;; int64 _y2
    8
    ;; int64 _x3
    8
    ;; int64 _y3
    8
    ;; int64 _x4
    8
    ;; int64 _y4
    8
    ;; int64 _x5
    8
    ;; int64 _y5
    8
    ;; int64 _x6
    8
    ;; int64 _y6
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int64 _x1
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _x1 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _x1) (= (length (_x1 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_x1 . bv) 0) 0) s)
              (write-long (ash (elt (_x1 . bv) 1) -1) s))
             ((and (class _x1) (= (length (_x1 . bv)) 1)) ;; big1
              (write-long (elt (_x1 . bv) 0) s)
              (write-long (if (>= _x1 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _x1 s)(write-long (if (>= _x1 0) 0 #xffffffff) s)))
     ;; int64 _y1
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _y1 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _y1) (= (length (_y1 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_y1 . bv) 0) 0) s)
              (write-long (ash (elt (_y1 . bv) 1) -1) s))
             ((and (class _y1) (= (length (_y1 . bv)) 1)) ;; big1
              (write-long (elt (_y1 . bv) 0) s)
              (write-long (if (>= _y1 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _y1 s)(write-long (if (>= _y1 0) 0 #xffffffff) s)))
     ;; int64 _x2
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _x2 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _x2) (= (length (_x2 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_x2 . bv) 0) 0) s)
              (write-long (ash (elt (_x2 . bv) 1) -1) s))
             ((and (class _x2) (= (length (_x2 . bv)) 1)) ;; big1
              (write-long (elt (_x2 . bv) 0) s)
              (write-long (if (>= _x2 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _x2 s)(write-long (if (>= _x2 0) 0 #xffffffff) s)))
     ;; int64 _y2
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _y2 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _y2) (= (length (_y2 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_y2 . bv) 0) 0) s)
              (write-long (ash (elt (_y2 . bv) 1) -1) s))
             ((and (class _y2) (= (length (_y2 . bv)) 1)) ;; big1
              (write-long (elt (_y2 . bv) 0) s)
              (write-long (if (>= _y2 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _y2 s)(write-long (if (>= _y2 0) 0 #xffffffff) s)))
     ;; int64 _x3
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _x3 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _x3) (= (length (_x3 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_x3 . bv) 0) 0) s)
              (write-long (ash (elt (_x3 . bv) 1) -1) s))
             ((and (class _x3) (= (length (_x3 . bv)) 1)) ;; big1
              (write-long (elt (_x3 . bv) 0) s)
              (write-long (if (>= _x3 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _x3 s)(write-long (if (>= _x3 0) 0 #xffffffff) s)))
     ;; int64 _y3
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _y3 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _y3) (= (length (_y3 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_y3 . bv) 0) 0) s)
              (write-long (ash (elt (_y3 . bv) 1) -1) s))
             ((and (class _y3) (= (length (_y3 . bv)) 1)) ;; big1
              (write-long (elt (_y3 . bv) 0) s)
              (write-long (if (>= _y3 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _y3 s)(write-long (if (>= _y3 0) 0 #xffffffff) s)))
     ;; int64 _x4
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _x4 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _x4) (= (length (_x4 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_x4 . bv) 0) 0) s)
              (write-long (ash (elt (_x4 . bv) 1) -1) s))
             ((and (class _x4) (= (length (_x4 . bv)) 1)) ;; big1
              (write-long (elt (_x4 . bv) 0) s)
              (write-long (if (>= _x4 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _x4 s)(write-long (if (>= _x4 0) 0 #xffffffff) s)))
     ;; int64 _y4
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _y4 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _y4) (= (length (_y4 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_y4 . bv) 0) 0) s)
              (write-long (ash (elt (_y4 . bv) 1) -1) s))
             ((and (class _y4) (= (length (_y4 . bv)) 1)) ;; big1
              (write-long (elt (_y4 . bv) 0) s)
              (write-long (if (>= _y4 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _y4 s)(write-long (if (>= _y4 0) 0 #xffffffff) s)))
     ;; int64 _x5
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _x5 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _x5) (= (length (_x5 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_x5 . bv) 0) 0) s)
              (write-long (ash (elt (_x5 . bv) 1) -1) s))
             ((and (class _x5) (= (length (_x5 . bv)) 1)) ;; big1
              (write-long (elt (_x5 . bv) 0) s)
              (write-long (if (>= _x5 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _x5 s)(write-long (if (>= _x5 0) 0 #xffffffff) s)))
     ;; int64 _y5
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _y5 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _y5) (= (length (_y5 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_y5 . bv) 0) 0) s)
              (write-long (ash (elt (_y5 . bv) 1) -1) s))
             ((and (class _y5) (= (length (_y5 . bv)) 1)) ;; big1
              (write-long (elt (_y5 . bv) 0) s)
              (write-long (if (>= _y5 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _y5 s)(write-long (if (>= _y5 0) 0 #xffffffff) s)))
     ;; int64 _x6
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _x6 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _x6) (= (length (_x6 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_x6 . bv) 0) 0) s)
              (write-long (ash (elt (_x6 . bv) 1) -1) s))
             ((and (class _x6) (= (length (_x6 . bv)) 1)) ;; big1
              (write-long (elt (_x6 . bv) 0) s)
              (write-long (if (>= _x6 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _x6 s)(write-long (if (>= _x6 0) 0 #xffffffff) s)))
     ;; int64 _y6
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _y6 (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _y6) (= (length (_y6 . bv)) 2)) ;; bignum
              (write-long (ash (elt (_y6 . bv) 0) 0) s)
              (write-long (ash (elt (_y6 . bv) 1) -1) s))
             ((and (class _y6) (= (length (_y6 . bv)) 1)) ;; big1
              (write-long (elt (_y6 . bv) 0) s)
              (write-long (if (>= _y6 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _y6 s)(write-long (if (>= _y6 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int64 _x1
#+(or :alpha :irix6 :x86_64)
      (setf _x1 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _x1 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _y1
#+(or :alpha :irix6 :x86_64)
      (setf _y1 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _y1 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _x2
#+(or :alpha :irix6 :x86_64)
      (setf _x2 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _x2 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _y2
#+(or :alpha :irix6 :x86_64)
      (setf _y2 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _y2 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _x3
#+(or :alpha :irix6 :x86_64)
      (setf _x3 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _x3 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _y3
#+(or :alpha :irix6 :x86_64)
      (setf _y3 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _y3 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _x4
#+(or :alpha :irix6 :x86_64)
      (setf _x4 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _x4 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _y4
#+(or :alpha :irix6 :x86_64)
      (setf _y4 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _y4 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _x5
#+(or :alpha :irix6 :x86_64)
      (setf _x5 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _x5 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _y5
#+(or :alpha :irix6 :x86_64)
      (setf _y5 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _y5 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _x6
#+(or :alpha :irix6 :x86_64)
      (setf _x6 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _x6 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _y6
#+(or :alpha :irix6 :x86_64)
      (setf _y6 (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _y6 (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get tip::Vector :md5sum-) "db63733935ea0547529d8e2d68307f12")
(setf (get tip::Vector :datatype-) "tip/Vector")
(setf (get tip::Vector :definition-)
      "Header header
int64 x1
int64 y1
int64 x2
int64 y2
int64 x3
int64 y3
int64 x4
int64 y4
int64 x5
int64 y5
int64 x6
int64 y6

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



(provide :tip/Vector "db63733935ea0547529d8e2d68307f12")


