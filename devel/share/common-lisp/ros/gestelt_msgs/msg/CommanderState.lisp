; Auto-generated. Do not edit!


(cl:in-package gestelt_msgs-msg)


;//! \htmlinclude CommanderState.msg.html

(cl:defclass <CommanderState> (roslisp-msg-protocol:ros-message)
  ((drone_id
    :reader drone_id
    :initarg :drone_id
    :type cl:integer
    :initform 0)
   (traj_server_state
    :reader traj_server_state
    :initarg :traj_server_state
    :type cl:string
    :initform "")
   (planner_server_state
    :reader planner_server_state
    :initarg :planner_server_state
    :type cl:string
    :initform "")
   (uav_state
    :reader uav_state
    :initarg :uav_state
    :type cl:string
    :initform "")
   (armed
    :reader armed
    :initarg :armed
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CommanderState (<CommanderState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommanderState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommanderState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gestelt_msgs-msg:<CommanderState> is deprecated: use gestelt_msgs-msg:CommanderState instead.")))

(cl:ensure-generic-function 'drone_id-val :lambda-list '(m))
(cl:defmethod drone_id-val ((m <CommanderState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:drone_id-val is deprecated.  Use gestelt_msgs-msg:drone_id instead.")
  (drone_id m))

(cl:ensure-generic-function 'traj_server_state-val :lambda-list '(m))
(cl:defmethod traj_server_state-val ((m <CommanderState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:traj_server_state-val is deprecated.  Use gestelt_msgs-msg:traj_server_state instead.")
  (traj_server_state m))

(cl:ensure-generic-function 'planner_server_state-val :lambda-list '(m))
(cl:defmethod planner_server_state-val ((m <CommanderState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:planner_server_state-val is deprecated.  Use gestelt_msgs-msg:planner_server_state instead.")
  (planner_server_state m))

(cl:ensure-generic-function 'uav_state-val :lambda-list '(m))
(cl:defmethod uav_state-val ((m <CommanderState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:uav_state-val is deprecated.  Use gestelt_msgs-msg:uav_state instead.")
  (uav_state m))

(cl:ensure-generic-function 'armed-val :lambda-list '(m))
(cl:defmethod armed-val ((m <CommanderState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:armed-val is deprecated.  Use gestelt_msgs-msg:armed instead.")
  (armed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommanderState>) ostream)
  "Serializes a message object of type '<CommanderState>"
  (cl:let* ((signed (cl:slot-value msg 'drone_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'traj_server_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'traj_server_state))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'planner_server_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'planner_server_state))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'uav_state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'uav_state))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'armed) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommanderState>) istream)
  "Deserializes a message object of type '<CommanderState>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drone_id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'traj_server_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'traj_server_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'planner_server_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'planner_server_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'uav_state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'uav_state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'armed) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommanderState>)))
  "Returns string type for a message object of type '<CommanderState>"
  "gestelt_msgs/CommanderState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommanderState)))
  "Returns string type for a message object of type 'CommanderState"
  "gestelt_msgs/CommanderState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommanderState>)))
  "Returns md5sum for a message object of type '<CommanderState>"
  "b2808123e870beb3f216ec9c58781e6e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommanderState)))
  "Returns md5sum for a message object of type 'CommanderState"
  "b2808123e870beb3f216ec9c58781e6e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommanderState>)))
  "Returns full string definition for message of type '<CommanderState>"
  (cl:format cl:nil "int32 drone_id~%string traj_server_state~%string planner_server_state~%string uav_state~%bool armed~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommanderState)))
  "Returns full string definition for message of type 'CommanderState"
  (cl:format cl:nil "int32 drone_id~%string traj_server_state~%string planner_server_state~%string uav_state~%bool armed~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommanderState>))
  (cl:+ 0
     4
     4 (cl:length (cl:slot-value msg 'traj_server_state))
     4 (cl:length (cl:slot-value msg 'planner_server_state))
     4 (cl:length (cl:slot-value msg 'uav_state))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommanderState>))
  "Converts a ROS message object to a list"
  (cl:list 'CommanderState
    (cl:cons ':drone_id (drone_id msg))
    (cl:cons ':traj_server_state (traj_server_state msg))
    (cl:cons ':planner_server_state (planner_server_state msg))
    (cl:cons ':uav_state (uav_state msg))
    (cl:cons ':armed (armed msg))
))
