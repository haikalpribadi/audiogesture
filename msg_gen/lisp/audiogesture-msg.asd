
(cl:in-package :asdf)

(defsystem "audiogesture-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TrainerStatus" :depends-on ("_package_TrainerStatus"))
    (:file "_package_TrainerStatus" :depends-on ("_package"))
    (:file "ExtractorStatus" :depends-on ("_package_ExtractorStatus"))
    (:file "_package_ExtractorStatus" :depends-on ("_package"))
    (:file "FeatureVector" :depends-on ("_package_FeatureVector"))
    (:file "_package_FeatureVector" :depends-on ("_package"))
    (:file "Strings" :depends-on ("_package_Strings"))
    (:file "_package_Strings" :depends-on ("_package"))
    (:file "ProcessedOutput" :depends-on ("_package_ProcessedOutput"))
    (:file "_package_ProcessedOutput" :depends-on ("_package"))
  ))