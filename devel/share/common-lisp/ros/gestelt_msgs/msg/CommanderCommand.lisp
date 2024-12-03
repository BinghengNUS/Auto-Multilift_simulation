; Auto-generated. Do not edit!


(cl:in-package gestelt_msgs-msg)


;//! \htmlinclude CommanderCommand.msg.html

(cl:defclass <CommanderCommand> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CommanderCommand (<CommanderCommand>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommanderCommand>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommanderCommand)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gestelt_msgs-msg:<CommanderCommand> is deprecated: use gestelt_msgs-msg:CommanderCommand instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CommanderCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:header-val is deprecated.  Use gestelt_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <CommanderCommand>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:command-val is deprecated.  Use gestelt_msgs-msg:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CommanderCommand>)))
    "Constants for message type '<CommanderCommand>"
  '((:TAKEOFF . 0)
    (:LAND . 1)
    (:MISSION . 2)
    (:HOVER . 3)
    (:E_STOP . 4)
    (:EMPTY . 5))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CommanderCommand)))
    "Constants for message type 'CommanderCommand"
  '((:TAKEOFF . 0)
    (:LAND . 1)
    (:MISSION . 2)
    (:HOVER . 3)
    (:E_STOP . 4)
    (:EMPTY . 5))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommanderCommand>) ostream)
  "Serializes a message object of type '<CommanderCommand>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommanderCommand>) istream)
  "Deserializes a message object of type '<CommanderCommand>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'command)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'command)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommanderCommand>)))
  "Returns string type for a message object of type '<CommanderCommand>"
  "gestelt_msgs/CommanderCommand")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommanderCommand)))
  "Returns string type for a message object of type 'CommanderCommand"
  "gestelt_msgs/CommanderCommand")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommanderCommand>)))
  "Returns md5sum for a message object of type '<CommanderCommand>"
  "d2b5cea46d5563515ff555ff17901255")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommanderCommand)))
  "Returns md5sum for a message object of type 'CommanderCommand"
  "d2b5cea46d5563515ff555ff17901255")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommanderCommand>)))
  "Returns full string definition for message of type '<CommanderCommand>"
  (cl:format cl:nil "std_msgs/Header header~%~%# Command sent to server ~%uint16 command~%uint16 TAKEOFF = 0~%uint16 LAND = 1~%uint16 MISSION = 2~%uint16 HOVER = 3~%uint16 E_STOP = 4~%uint16 EMPTY = 5~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommanderCommand)))
  "Returns full string definition for message of type 'CommanderCommand"
  (cl:format cl:nil "std_msgs/Header header~%~%# Command sent to server ~%uint16 command~%uint16 TAKEOFF = 0~%uint16 LAND = 1~%uint16 MISSION = 2~%uint16 HOVER = 3~%uint16 E_STOP = 4~%uint16 EMPTY = 5~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommanderCommand>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommanderCommand>))
  "Converts a ROS message object to a list"
  (cl:list 'CommanderCommand
    (cl:cons ':header (header msg))
    (cl:cons ':command (command msg))
))
