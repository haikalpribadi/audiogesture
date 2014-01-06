
(cl:in-package :asdf)

(defsystem "audiogesture-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Bextractor" :depends-on ("_package_Bextractor"))
    (:file "_package_Bextractor" :depends-on ("_package"))
  ))