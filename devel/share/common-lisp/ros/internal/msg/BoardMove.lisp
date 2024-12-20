; Auto-generated. Do not edit!


(cl:in-package internal-msg)


;//! \htmlinclude BoardMove.msg.html

(cl:defclass <BoardMove> (roslisp-msg-protocol:ros-message)
  ((start_x
    :reader start_x
    :initarg :start_x
    :type cl:integer
    :initform 0)
   (start_y
    :reader start_y
    :initarg :start_y
    :type cl:integer
    :initform 0)
   (end_x
    :reader end_x
    :initarg :end_x
    :type cl:integer
    :initform 0)
   (end_y
    :reader end_y
    :initarg :end_y
    :type cl:integer
    :initform 0))
)

(cl:defclass BoardMove (<BoardMove>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoardMove>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoardMove)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name internal-msg:<BoardMove> is deprecated: use internal-msg:BoardMove instead.")))

(cl:ensure-generic-function 'start_x-val :lambda-list '(m))
(cl:defmethod start_x-val ((m <BoardMove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader internal-msg:start_x-val is deprecated.  Use internal-msg:start_x instead.")
  (start_x m))

(cl:ensure-generic-function 'start_y-val :lambda-list '(m))
(cl:defmethod start_y-val ((m <BoardMove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader internal-msg:start_y-val is deprecated.  Use internal-msg:start_y instead.")
  (start_y m))

(cl:ensure-generic-function 'end_x-val :lambda-list '(m))
(cl:defmethod end_x-val ((m <BoardMove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader internal-msg:end_x-val is deprecated.  Use internal-msg:end_x instead.")
  (end_x m))

(cl:ensure-generic-function 'end_y-val :lambda-list '(m))
(cl:defmethod end_y-val ((m <BoardMove>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader internal-msg:end_y-val is deprecated.  Use internal-msg:end_y instead.")
  (end_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoardMove>) ostream)
  "Serializes a message object of type '<BoardMove>"
  (cl:let* ((signed (cl:slot-value msg 'start_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'start_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'end_x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'end_y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoardMove>) istream)
  "Deserializes a message object of type '<BoardMove>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'start_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'end_x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'end_y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoardMove>)))
  "Returns string type for a message object of type '<BoardMove>"
  "internal/BoardMove")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoardMove)))
  "Returns string type for a message object of type 'BoardMove"
  "internal/BoardMove")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoardMove>)))
  "Returns md5sum for a message object of type '<BoardMove>"
  "2d9a81541f0e7558640a92686275d893")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoardMove)))
  "Returns md5sum for a message object of type 'BoardMove"
  "2d9a81541f0e7558640a92686275d893")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoardMove>)))
  "Returns full string definition for message of type '<BoardMove>"
  (cl:format cl:nil "int32 start_x~%int32 start_y~%int32 end_x~%int32 end_y~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoardMove)))
  "Returns full string definition for message of type 'BoardMove"
  (cl:format cl:nil "int32 start_x~%int32 start_y~%int32 end_x~%int32 end_y~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoardMove>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoardMove>))
  "Converts a ROS message object to a list"
  (cl:list 'BoardMove
    (cl:cons ':start_x (start_x msg))
    (cl:cons ':start_y (start_y msg))
    (cl:cons ':end_x (end_x msg))
    (cl:cons ':end_y (end_y msg))
))
