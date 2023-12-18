;; Auto-generated. Do not edit!


(when (boundp 'jog_msgs::JogFrame)
  (if (not (find-package "JOG_MSGS"))
    (make-package "JOG_MSGS"))
  (shadow 'JogFrame (find-package "JOG_MSGS")))
(unless (find-package "JOG_MSGS::JOGFRAME")
  (make-package "JOG_MSGS::JOGFRAME"))

(in-package "ROS")
;;//! \htmlinclude JogFrame.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass jog_msgs::JogFrame
  :super ros::object
  :slots (_header _group_name _link_name _linear_delta _angular_delta _avoid_collisions ))

(defmethod jog_msgs::JogFrame
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:group_name __group_name) "")
    ((:link_name __link_name) "")
    ((:linear_delta __linear_delta) (instance geometry_msgs::Vector3 :init))
    ((:angular_delta __angular_delta) (instance geometry_msgs::Vector3 :init))
    ((:avoid_collisions __avoid_collisions) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _group_name (string __group_name))
   (setq _link_name (string __link_name))
   (setq _linear_delta __linear_delta)
   (setq _angular_delta __angular_delta)
   (setq _avoid_collisions __avoid_collisions)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:group_name
   (&optional __group_name)
   (if __group_name (setq _group_name __group_name)) _group_name)
  (:link_name
   (&optional __link_name)
   (if __link_name (setq _link_name __link_name)) _link_name)
  (:linear_delta
   (&rest __linear_delta)
   (if (keywordp (car __linear_delta))
       (send* _linear_delta __linear_delta)
     (progn
       (if __linear_delta (setq _linear_delta (car __linear_delta)))
       _linear_delta)))
  (:angular_delta
   (&rest __angular_delta)
   (if (keywordp (car __angular_delta))
       (send* _angular_delta __angular_delta)
     (progn
       (if __angular_delta (setq _angular_delta (car __angular_delta)))
       _angular_delta)))
  (:avoid_collisions
   (&optional __avoid_collisions)
   (if __avoid_collisions (setq _avoid_collisions __avoid_collisions)) _avoid_collisions)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _group_name
    4 (length _group_name)
    ;; string _link_name
    4 (length _link_name)
    ;; geometry_msgs/Vector3 _linear_delta
    (send _linear_delta :serialization-length)
    ;; geometry_msgs/Vector3 _angular_delta
    (send _angular_delta :serialization-length)
    ;; bool _avoid_collisions
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _group_name
       (write-long (length _group_name) s) (princ _group_name s)
     ;; string _link_name
       (write-long (length _link_name) s) (princ _link_name s)
     ;; geometry_msgs/Vector3 _linear_delta
       (send _linear_delta :serialize s)
     ;; geometry_msgs/Vector3 _angular_delta
       (send _angular_delta :serialize s)
     ;; bool _avoid_collisions
       (if _avoid_collisions (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _group_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _group_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _link_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _link_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Vector3 _linear_delta
     (send _linear_delta :deserialize buf ptr-) (incf ptr- (send _linear_delta :serialization-length))
   ;; geometry_msgs/Vector3 _angular_delta
     (send _angular_delta :deserialize buf ptr-) (incf ptr- (send _angular_delta :serialization-length))
   ;; bool _avoid_collisions
     (setq _avoid_collisions (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get jog_msgs::JogFrame :md5sum-) "e342f29bf6beaf00261bdae365abfff9")
(setf (get jog_msgs::JogFrame :datatype-) "jog_msgs/JogFrame")
(setf (get jog_msgs::JogFrame :definition-)
      "# This is a message to hold data to jog by specifying a target
# frame. It uses MoveIt! kinematics, so you need to specify the
# JointGroup name to use in group_name. (lienar|angular)_delta is the
# amount of displacement.

# header message. You must set frame_id to define the reference
# coordinate system of the displacament
Header header

# Name of JointGroup of MoveIt!
string group_name

# Target link name to jog. The link must be in the JoingGroup
string link_name

# Linear displacement vector to jog. The refrence frame is defined by
# frame_id in header. Unit is in meter.
geometry_msgs/Vector3 linear_delta

# Angular displacement vector to jog. The refrence frame is defined by
# frame_id in header. Unit is in radian.
geometry_msgs/Vector3 angular_delta

# It uses avoid_collisions option of MoveIt! kinematics. If it is
# true, the robot doesn't move if any collisions occured.
bool avoid_collisions

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
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :jog_msgs/JogFrame "e342f29bf6beaf00261bdae365abfff9")


