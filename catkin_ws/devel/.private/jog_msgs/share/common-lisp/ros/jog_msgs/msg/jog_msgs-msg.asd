
(cl:in-package :asdf)

(defsystem "jog_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "JogFrame" :depends-on ("_package_JogFrame"))
    (:file "_package_JogFrame" :depends-on ("_package"))
    (:file "JogJoint" :depends-on ("_package_JogJoint"))
    (:file "_package_JogJoint" :depends-on ("_package"))
  ))