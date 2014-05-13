
(cl:in-package :asdf)

(defsystem "agvc_gps-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "ConvertGPSOrigin" :depends-on ("_package_ConvertGPSOrigin"))
    (:file "_package_ConvertGPSOrigin" :depends-on ("_package"))
  ))