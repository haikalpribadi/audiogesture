
(cl:in-package :asdf)

(defsystem "audiogesture-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetFile" :depends-on ("_package_GetFile"))
    (:file "_package_GetFile" :depends-on ("_package"))
    (:file "GetSamples" :depends-on ("_package_GetSamples"))
    (:file "_package_GetSamples" :depends-on ("_package"))
  ))