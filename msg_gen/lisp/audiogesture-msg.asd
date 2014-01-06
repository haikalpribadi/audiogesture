
(cl:in-package :asdf)

(defsystem "audiogesture-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Strings" :depends-on ("_package_Strings"))
    (:file "_package_Strings" :depends-on ("_package"))
  ))