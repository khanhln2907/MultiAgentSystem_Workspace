
(cl:in-package :asdf)

(defsystem "voronoi_draw-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CentralizedMsg" :depends-on ("_package_CentralizedMsg"))
    (:file "_package_CentralizedMsg" :depends-on ("_package"))
  ))