;; Auto-generated. Do not edit!


(when (boundp 'jog_msgs::JogJoint)
  (if (not (find-package "JOG_MSGS"))
    (make-package "JOG_MSGS"))
  (shadow 'JogJoint (find-package "JOG_MSGS")))
(unless (find-package "JOG_MSGS::JOGJOINT")
  (make-package "JOG_MSGS::JOGJOINT"))

(in-package "ROS")
;;//! \htmlinclude JogJoint.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jog_msgs::JogJoint
  :super ros::object
  :slots (_header _joint_names _deltas ))

(defmethod jog_msgs::JogJoint
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_names __joint_names) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:deltas __deltas) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_names __joint_names)
   (setq _deltas __deltas)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_names
   (&optional __joint_names)
   (if __joint_names (setq _joint_names __joint_names)) _joint_names)
  (:deltas
   (&optional __deltas)
   (if __deltas (setq _deltas __deltas)) _deltas)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string[] _joint_names
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _joint_names)) 4
    ;; float64[] _deltas
    (* 8    (length _deltas)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string[] _joint_names
     (write-long (length _joint_names) s)
     (dolist (elem _joint_names)
       (write-long (length elem) s) (princ elem s)
       )
     ;; float64[] _deltas
     (write-long (length _deltas) s)
     (dotimes (i (length _deltas))
       (sys::poke (elt _deltas i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string[] _joint_names
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_names (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _joint_names i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; float64[] _deltas
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _deltas (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _deltas i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get jog_msgs::JogJoint :md5sum-) "8d2aa14be64b51cf6374d198bfd489b2")
(setf (get jog_msgs::JogJoint :datatype-) "jog_msgs/JogJoint")
(setf (get jog_msgs::JogJoint :definition-)
      "# This is a message to hold data to jog by specifying joint
# displacement. You only need to set relative displacement to joint
# angles (or displacements for linear joints).

# header message. You must set frame_id to define the reference
# coordinate system of the displacament
Header header

# Name list of the joints. You don't need to specify all joint of the
# robot. Joint names are case-sensitive.
string[] joint_names

# Relative displacement of the joints to jog. The order must be
# identical to joint_names. Unit is in radian for revolutive joints,
# meter for linear joints.
float64[] deltas

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



(provide :jog_msgs/JogJoint "8d2aa14be64b51cf6374d198bfd489b2")


