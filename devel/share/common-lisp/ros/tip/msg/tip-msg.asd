
(cl:in-package :asdf)

(defsystem "tip-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ControlMsg" :depends-on ("_package_ControlMsg"))
    (:file "_package_ControlMsg" :depends-on ("_package"))
    (:file "UnicycleInfoMsg" :depends-on ("_package_UnicycleInfoMsg"))
    (:file "_package_UnicycleInfoMsg" :depends-on ("_package"))
    (:file "UnicycleInfoStruct" :depends-on ("_package_UnicycleInfoStruct"))
    (:file "_package_UnicycleInfoStruct" :depends-on ("_package"))
    (:file "Vector" :depends-on ("_package_Vector"))
    (:file "_package_Vector" :depends-on ("_package"))
  ))