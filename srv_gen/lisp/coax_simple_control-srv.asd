
(cl:in-package :asdf)

(defsystem "coax_simple_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetNavMode" :depends-on ("_package_SetNavMode"))
    (:file "_package_SetNavMode" :depends-on ("_package"))
    (:file "SetControlMode" :depends-on ("_package_SetControlMode"))
    (:file "_package_SetControlMode" :depends-on ("_package"))
    (:file "SetWaypoint" :depends-on ("_package_SetWaypoint"))
    (:file "_package_SetWaypoint" :depends-on ("_package"))
  ))