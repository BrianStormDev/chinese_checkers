;; Auto-generated. Do not edit!


(when (boundp 'internal::BoardMove)
  (if (not (find-package "INTERNAL"))
    (make-package "INTERNAL"))
  (shadow 'BoardMove (find-package "INTERNAL")))
(unless (find-package "INTERNAL::BOARDMOVE")
  (make-package "INTERNAL::BOARDMOVE"))

(in-package "ROS")
;;//! \htmlinclude BoardMove.msg.html


(defclass internal::BoardMove
  :super ros::object
  :slots (_start_x _start_y _end_x _end_y ))

(defmethod internal::BoardMove
  (:init
   (&key
    ((:start_x __start_x) 0)
    ((:start_y __start_y) 0)
    ((:end_x __end_x) 0)
    ((:end_y __end_y) 0)
    )
   (send-super :init)
   (setq _start_x (round __start_x))
   (setq _start_y (round __start_y))
   (setq _end_x (round __end_x))
   (setq _end_y (round __end_y))
   self)
  (:start_x
   (&optional __start_x)
   (if __start_x (setq _start_x __start_x)) _start_x)
  (:start_y
   (&optional __start_y)
   (if __start_y (setq _start_y __start_y)) _start_y)
  (:end_x
   (&optional __end_x)
   (if __end_x (setq _end_x __end_x)) _end_x)
  (:end_y
   (&optional __end_y)
   (if __end_y (setq _end_y __end_y)) _end_y)
  (:serialization-length
   ()
   (+
    ;; int32 _start_x
    4
    ;; int32 _start_y
    4
    ;; int32 _end_x
    4
    ;; int32 _end_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _start_x
       (write-long _start_x s)
     ;; int32 _start_y
       (write-long _start_y s)
     ;; int32 _end_x
       (write-long _end_x s)
     ;; int32 _end_y
       (write-long _end_y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _start_x
     (setq _start_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _start_y
     (setq _start_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _end_x
     (setq _end_x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _end_y
     (setq _end_y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get internal::BoardMove :md5sum-) "2d9a81541f0e7558640a92686275d893")
(setf (get internal::BoardMove :datatype-) "internal/BoardMove")
(setf (get internal::BoardMove :definition-)
      "int32 start_x
int32 start_y
int32 end_x
int32 end_y
")



(provide :internal/BoardMove "2d9a81541f0e7558640a92686275d893")


