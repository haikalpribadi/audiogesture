
(cl:in-package :asdf)

(defsystem "audiogesture-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "GetSamples" :depends-on ("_package_GetSamples"))
    (:file "_package_GetSamples" :depends-on ("_package"))
    (:file "GetSampleFile" :depends-on ("_package_GetSampleFile"))
    (:file "_package_GetSampleFile" :depends-on ("_package"))
  ))