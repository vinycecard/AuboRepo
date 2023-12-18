; Auto-generated. Do not edit!


(cl:in-package jog_msgs-msg)


;//! \htmlinclude JogJoint.msg.html

(cl:defclass <JogJoint> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (joint_names
    :reader joint_names
    :initarg :joint_names
    :type (cl:vector cl:string)
   :initform (cl:make-array 0 :element-type 'cl:string :initial-element ""))
   (deltas
    :reader deltas
    :initarg :deltas
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass JogJoint (<JogJoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JogJoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JogJoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name jog_msgs-msg:<JogJoint> is deprecated: use jog_msgs-msg:JogJoint instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <JogJoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jog_msgs-msg:header-val is deprecated.  Use jog_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'joint_names-val :lambda-list '(m))
(cl:defmethod joint_names-val ((m <JogJoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jog_msgs-msg:joint_names-val is deprecated.  Use jog_msgs-msg:joint_names instead.")
  (joint_names m))

(cl:ensure-generic-function 'deltas-val :lambda-list '(m))
(cl:defmethod deltas-val ((m <JogJoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader jog_msgs-msg:deltas-val is deprecated.  Use jog_msgs-msg:deltas instead.")
  (deltas m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JogJoint>) ostream)
  "Serializes a message object of type '<JogJoint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'joint_names))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((__ros_str_len (cl:length ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) ele))
   (cl:slot-value msg 'joint_names))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'deltas))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'deltas))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JogJoint>) istream)
  "Deserializes a message object of type '<JogJoint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'joint_names) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'joint_names)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:aref vals i) __ros_str_idx) (cl:code-char (cl:read-byte istream))))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'deltas) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'deltas)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JogJoint>)))
  "Returns string type for a message object of type '<JogJoint>"
  "jog_msgs/JogJoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JogJoint)))
  "Returns string type for a message object of type 'JogJoint"
  "jog_msgs/JogJoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JogJoint>)))
  "Returns md5sum for a message object of type '<JogJoint>"
  "8d2aa14be64b51cf6374d198bfd489b2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JogJoint)))
  "Returns md5sum for a message object of type 'JogJoint"
  "8d2aa14be64b51cf6374d198bfd489b2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JogJoint>)))
  "Returns full string definition for message of type '<JogJoint>"
  (cl:format cl:nil "# This is a message to hold data to jog by specifying joint~%# displacement. You only need to set relative displacement to joint~%# angles (or displacements for linear joints).~%~%# header message. You must set frame_id to define the reference~%# coordinate system of the displacament~%Header header~%~%# Name list of the joints. You don't need to specify all joint of the~%# robot. Joint names are case-sensitive.~%string[] joint_names~%~%# Relative displacement of the joints to jog. The order must be~%# identical to joint_names. Unit is in radian for revolutive joints,~%# meter for linear joints.~%float64[] deltas~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JogJoint)))
  "Returns full string definition for message of type 'JogJoint"
  (cl:format cl:nil "# This is a message to hold data to jog by specifying joint~%# displacement. You only need to set relative displacement to joint~%# angles (or displacements for linear joints).~%~%# header message. You must set frame_id to define the reference~%# coordinate system of the displacament~%Header header~%~%# Name list of the joints. You don't need to specify all joint of the~%# robot. Joint names are case-sensitive.~%string[] joint_names~%~%# Relative displacement of the joints to jog. The order must be~%# identical to joint_names. Unit is in radian for revolutive joints,~%# meter for linear joints.~%float64[] deltas~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JogJoint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'joint_names) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4 (cl:length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'deltas) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JogJoint>))
  "Converts a ROS message object to a list"
  (cl:list 'JogJoint
    (cl:cons ':header (header msg))
    (cl:cons ':joint_names (joint_names msg))
    (cl:cons ':deltas (deltas msg))
))
