
(cl:in-package :asdf)

(defsystem "internal-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "BoardMove" :depends-on ("_package_BoardMove"))
    (:file "_package_BoardMove" :depends-on ("_package"))
  ))