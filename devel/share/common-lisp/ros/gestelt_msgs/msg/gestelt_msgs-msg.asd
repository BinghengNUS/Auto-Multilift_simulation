
(cl:in-package :asdf)

(defsystem "gestelt_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "CommanderCommand" :depends-on ("_package_CommanderCommand"))
    (:file "_package_CommanderCommand" :depends-on ("_package"))
    (:file "CommanderState" :depends-on ("_package_CommanderState"))
    (:file "_package_CommanderState" :depends-on ("_package"))
    (:file "Goals" :depends-on ("_package_Goals"))
    (:file "_package_Goals" :depends-on ("_package"))
  ))