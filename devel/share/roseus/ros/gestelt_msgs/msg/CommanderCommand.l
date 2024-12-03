;; Auto-generated. Do not edit!


(when (boundp 'gestelt_msgs::CommanderCommand)
  (if (not (find-package "GESTELT_MSGS"))
    (make-package "GESTELT_MSGS"))
  (shadow 'CommanderCommand (find-package "GESTELT_MSGS")))
(unless (find-package "GESTELT_MSGS::COMMANDERCOMMAND")
  (make-package "GESTELT_MSGS::COMMANDERCOMMAND"))

(in-package "ROS")
;;//! \htmlinclude CommanderCommand.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*TAKEOFF*" (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(shadow '*TAKEOFF* (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(defconstant gestelt_msgs::CommanderCommand::*TAKEOFF* 0)
(intern "*LAND*" (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(shadow '*LAND* (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(defconstant gestelt_msgs::CommanderCommand::*LAND* 1)
(intern "*MISSION*" (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(shadow '*MISSION* (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(defconstant gestelt_msgs::CommanderCommand::*MISSION* 2)
(intern "*HOVER*" (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(shadow '*HOVER* (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(defconstant gestelt_msgs::CommanderCommand::*HOVER* 3)
(intern "*E_STOP*" (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(shadow '*E_STOP* (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(defconstant gestelt_msgs::CommanderCommand::*E_STOP* 4)
(intern "*EMPTY*" (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(shadow '*EMPTY* (find-package "GESTELT_MSGS::COMMANDERCOMMAND"))
(defconstant gestelt_msgs::CommanderCommand::*EMPTY* 5)

(defun gestelt_msgs::CommanderCommand-to-symbol (const)
  (cond
        ((= const 0) 'gestelt_msgs::CommanderCommand::*TAKEOFF*)
        ((= const 1) 'gestelt_msgs::CommanderCommand::*LAND*)
        ((= const 2) 'gestelt_msgs::CommanderCommand::*MISSION*)
        ((= const 3) 'gestelt_msgs::CommanderCommand::*HOVER*)
        ((= const 4) 'gestelt_msgs::CommanderCommand::*E_STOP*)
        ((= const 5) 'gestelt_msgs::CommanderCommand::*EMPTY*)
        (t nil)))

(defclass gestelt_msgs::CommanderCommand
  :super ros::object
  :slots (_header _command ))

(defmethod gestelt_msgs::CommanderCommand
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:command __command) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _command (round __command))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint16 _command
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint16 _command
       (write-word _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint16 _command
     (setq _command (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get gestelt_msgs::CommanderCommand :md5sum-) "d2b5cea46d5563515ff555ff17901255")
(setf (get gestelt_msgs::CommanderCommand :datatype-) "gestelt_msgs/CommanderCommand")
(setf (get gestelt_msgs::CommanderCommand :definition-)
      "std_msgs/Header header

# Command sent to server 
uint16 command
uint16 TAKEOFF = 0
uint16 LAND = 1
uint16 MISSION = 2
uint16 HOVER = 3
uint16 E_STOP = 4
uint16 EMPTY = 5

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :gestelt_msgs/CommanderCommand "d2b5cea46d5563515ff555ff17901255")


