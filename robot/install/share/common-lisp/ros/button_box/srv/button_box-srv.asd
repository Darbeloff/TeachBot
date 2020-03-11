
(cl:in-package :asdf)

(defsystem "button_box-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ButtonInfo" :depends-on ("_package_ButtonInfo"))
    (:file "_package_ButtonInfo" :depends-on ("_package"))
  ))