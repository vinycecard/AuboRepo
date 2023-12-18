; Auto-generated. Do not edit!


(cl:in-package jog_msgs-msg)


;//! \htmlinclude JogFrame.msg.html

(cl:defclass <JogFrame> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (group_name
    :reader group_name
    :initarg :group_name
    :type cl:string
    :initform "")
   (link_name
    :reader link_name
    :initarg :link_name
    :type cl:string
    :initform "")
   (linear_delta
    :reader linear_delta
    :initarg :linear_delta
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (angular_delta
    :reader angular_delta
    :initarg :angular_delta
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (avoid_collisions
    :reader avoid_collisions
    :initarg :avoid_collisions
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass JogFrame (<JogFrame>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JogFrame>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JogFrame)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jog_msgs-msg:<JogFrame> is deprecated: use jog_msgs-msg:JogFrame instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <JogFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jog_msgs-msg:header-val is deprecated.  Use jog_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'group_name-val :lambda-list '(m))
(cl:defmethod group_name-val ((m <JogFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jog_msgs-msg:group_name-val is deprecated.  Use jog_msgs-msg:group_name instead.")
  (group_name m))

(cl:ensure-generic-function 'link_name-val :lambda-list '(m))
(cl:defmethod link_name-val ((m <JogFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jog_msgs-msg:link_name-val is deprecated.  Use jog_msgs-msg:link_name instead.")
  (link_name m))

(cl:ensure-generic-function 'linear_delta-val :lambda-list '(m))
(cl:defmethod linear_delta-val ((m <JogFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jog_msgs-msg:linear_delta-val is deprecated.  Use jog_msgs-msg:linear_delta instead.")
  (linear_delta m))

(cl:ensure-generic-function 'angular_delta-val :lambda-list '(m))
(cl:defmethod angular_delta-val ((m <JogFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jog_msgs-msg:angular_delta-val is deprecated.  Use jog_msgs-msg:angular_delta instead.")
  (angular_delta m))

(cl:ensure-generic-function 'avoid_collisions-val :lambda-list '(m))
(cl:defmethod avoid_collisions-val ((m <JogFrame>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jog_msgs-msg:avoid_collisions-val is deprecated.  Use jog_msgs-msg:avoid_collisions instead.")
  (avoid_collisions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JogFrame>) ostream)
  "Serializes a message object of type '<JogFrame>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'group_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'group_name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'link_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'link_name))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'linear_delta) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular_delta) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'avoid_collisions) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JogFrame>) istream)
  "Deserializes a message object of type '<JogFrame>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'group_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'group_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'link_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'link_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'linear_delta) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular_delta) istream)
    (cl:setf (cl:slot-value msg 'avoid_collisions) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JogFrame>)))
  "Returns string type for a message object of type '<JogFrame>"
  "jog_msgs/JogFrame")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JogFrame)))
  "Returns string type for a message object of type 'JogFrame"
  "jog_msgs/JogFrame")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JogFrame>)))
  "Returns md5sum for a message object of type '<JogFrame>"
  "e342f29bf6beaf00261bdae365abfff9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JogFrame)))
  "Returns md5sum for a message object of type 'JogFrame"
  "e342f29bf6beaf00261bdae365abfff9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JogFrame>)))
  "Returns full string definition for message of type '<JogFrame>"
  (cl:format cl:nil "# This is a message to hold data to jog by specifying a target~%# frame. It uses MoveIt! kinematics, so you need to specify the~%# JointGroup name to use in group_name. (lienar|angular)_delta is the~%# amount of displacement.~%~%# header message. You must set frame_id to define the reference~%# coordinate system of the displacament~%Header header~%~%# Name of JointGroup of MoveIt!~%string group_name~%~%# Target link name to jog. The link must be in the JoingGroup~%string link_name~%~%# Linear displacement vector to jog. The refrence frame is defined by~%# frame_id in header. Unit is in meter.~%geometry_msgs/Vector3 linear_delta~%~%# Angular displacement vector to jog. The refrence frame is defined by~%# frame_id in header. Unit is in radian.~%geometry_msgs/Vector3 angular_delta~%~%# It uses avoid_collisions option of MoveIt! kinematics. If it is~%# true, the robot doesn't move if any collisions occured.~%bool avoid_collisions~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JogFrame)))
  "Returns full string definition for message of type 'JogFrame"
  (cl:format cl:nil "# This is a message to hold data to jog by specifying a target~%# frame. It uses MoveIt! kinematics, so you need to specify the~%# JointGroup name to use in group_name. (lienar|angular)_delta is the~%# amount of displacement.~%~%# header message. You must set frame_id to define the reference~%# coordinate system of the displacament~%Header header~%~%# Name of JointGroup of MoveIt!~%string group_name~%~%# Target link name to jog. The link must be in the JoingGroup~%string link_name~%~%# Linear displacement vector to jog. The refrence frame is defined by~%# frame_id in header. Unit is in meter.~%geometry_msgs/Vector3 linear_delta~%~%# Angular displacement vector to jog. The refrence frame is defined by~%# frame_id in header. Unit is in radian.~%geometry_msgs/Vector3 angular_delta~%~%# It uses avoid_collisions option of MoveIt! kinematics. If it is~%# true, the robot doesn't move if any collisions occured.~%bool avoid_collisions~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JogFrame>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:length (cl:slot-value msg 'group_name))
     4 (cl:length (cl:slot-value msg 'link_name))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'linear_delta))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular_delta))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JogFrame>))
  "Converts a ROS message object to a list"
  (cl:list 'JogFrame
    (cl:cons ':header (header msg))
    (cl:cons ':group_name (group_name msg))
    (cl:cons ':link_name (link_name msg))
    (cl:cons ':linear_delta (linear_delta msg))
    (cl:cons ':angular_delta (angular_delta msg))
    (cl:cons ':avoid_collisions (avoid_collisions msg))
))
