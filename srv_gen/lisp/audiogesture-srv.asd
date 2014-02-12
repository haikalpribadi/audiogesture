
(cl:in-package :asdf)

(defsystem "audiogesture-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "MusicExtractor" :depends-on ("_package_MusicExtractor"))
    (:file "_package_MusicExtractor" :depends-on ("_package"))
  ))