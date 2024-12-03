; Auto-generated. Do not edit!


(cl:in-package gestelt_msgs-msg)


;//! \htmlinclude Goals.msg.html

(cl:defclass <Goals> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector geometry_msgs-msg:Pose)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Pose :initial-element (cl:make-instance 'geometry_msgs-msg:Pose)))
   (velocities
    :reader velocities
    :initarg :velocities
    :type (cl:vector geometry_msgs-msg:Twist)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Twist :initial-element (cl:make-instance 'geometry_msgs-msg:Twist)))
   (accelerations
    :reader accelerations
    :initarg :accelerations
    :type (cl:vector geometry_msgs-msg:Accel)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Accel :initial-element (cl:make-instance 'geometry_msgs-msg:Accel)))
   (velocities_mask
    :reader velocities_mask
    :initarg :velocities_mask
    :type (cl:vector std_msgs-msg:Bool)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:Bool :initial-element (cl:make-instance 'std_msgs-msg:Bool)))
   (accelerations_mask
    :reader accelerations_mask
    :initarg :accelerations_mask
    :type (cl:vector std_msgs-msg:Bool)
   :initform (cl:make-array 0 :element-type 'std_msgs-msg:Bool :initial-element (cl:make-instance 'std_msgs-msg:Bool)))
   (time_factor_terminal
    :reader time_factor_terminal
    :initarg :time_factor_terminal
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (time_factor
    :reader time_factor
    :initarg :time_factor
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (max_vel
    :reader max_vel
    :initarg :max_vel
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (max_acc
    :reader max_acc
    :initarg :max_acc
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32)))
)

(cl:defclass Goals (<Goals>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Goals>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Goals)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gestelt_msgs-msg:<Goals> is deprecated: use gestelt_msgs-msg:Goals instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Goals>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:header-val is deprecated.  Use gestelt_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <Goals>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:waypoints-val is deprecated.  Use gestelt_msgs-msg:waypoints instead.")
  (waypoints m))

(cl:ensure-generic-function 'velocities-val :lambda-list '(m))
(cl:defmethod velocities-val ((m <Goals>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:velocities-val is deprecated.  Use gestelt_msgs-msg:velocities instead.")
  (velocities m))

(cl:ensure-generic-function 'accelerations-val :lambda-list '(m))
(cl:defmethod accelerations-val ((m <Goals>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:accelerations-val is deprecated.  Use gestelt_msgs-msg:accelerations instead.")
  (accelerations m))

(cl:ensure-generic-function 'velocities_mask-val :lambda-list '(m))
(cl:defmethod velocities_mask-val ((m <Goals>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:velocities_mask-val is deprecated.  Use gestelt_msgs-msg:velocities_mask instead.")
  (velocities_mask m))

(cl:ensure-generic-function 'accelerations_mask-val :lambda-list '(m))
(cl:defmethod accelerations_mask-val ((m <Goals>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:accelerations_mask-val is deprecated.  Use gestelt_msgs-msg:accelerations_mask instead.")
  (accelerations_mask m))

(cl:ensure-generic-function 'time_factor_terminal-val :lambda-list '(m))
(cl:defmethod time_factor_terminal-val ((m <Goals>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:time_factor_terminal-val is deprecated.  Use gestelt_msgs-msg:time_factor_terminal instead.")
  (time_factor_terminal m))

(cl:ensure-generic-function 'time_factor-val :lambda-list '(m))
(cl:defmethod time_factor-val ((m <Goals>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:time_factor-val is deprecated.  Use gestelt_msgs-msg:time_factor instead.")
  (time_factor m))

(cl:ensure-generic-function 'max_vel-val :lambda-list '(m))
(cl:defmethod max_vel-val ((m <Goals>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:max_vel-val is deprecated.  Use gestelt_msgs-msg:max_vel instead.")
  (max_vel m))

(cl:ensure-generic-function 'max_acc-val :lambda-list '(m))
(cl:defmethod max_acc-val ((m <Goals>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gestelt_msgs-msg:max_acc-val is deprecated.  Use gestelt_msgs-msg:max_acc instead.")
  (max_acc m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Goals>) ostream)
  "Serializes a message object of type '<Goals>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'velocities))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'velocities))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'accelerations))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'accelerations))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'velocities_mask))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'velocities_mask))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'accelerations_mask))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'accelerations_mask))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'time_factor_terminal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'time_factor) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max_vel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max_acc) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Goals>) istream)
  "Deserializes a message object of type '<Goals>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Pose))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'velocities) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'velocities)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Twist))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'accelerations) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'accelerations)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Accel))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'velocities_mask) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'velocities_mask)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:Bool))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'accelerations_mask) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'accelerations_mask)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'std_msgs-msg:Bool))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'time_factor_terminal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'time_factor) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max_vel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max_acc) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Goals>)))
  "Returns string type for a message object of type '<Goals>"
  "gestelt_msgs/Goals")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Goals)))
  "Returns string type for a message object of type 'Goals"
  "gestelt_msgs/Goals")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Goals>)))
  "Returns md5sum for a message object of type '<Goals>"
  "177ff0121ddb3a9dfb30d4b1b6d4fd32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Goals)))
  "Returns md5sum for a message object of type 'Goals"
  "177ff0121ddb3a9dfb30d4b1b6d4fd32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Goals>)))
  "Returns full string definition for message of type '<Goals>"
  (cl:format cl:nil "std_msgs/Header header~%~%# Goal waypoints~%geometry_msgs/Pose[] waypoints~%geometry_msgs/Twist[] velocities~%geometry_msgs/Accel[] accelerations~%~%std_msgs/Bool[] velocities_mask  # 0 means use the value, 1 means ignore the value~%std_msgs/Bool[] accelerations_mask # 0 means use the value, 1 means ignore the value~%~%std_msgs/Float32 time_factor_terminal~%std_msgs/Float32 time_factor~%std_msgs/Float32 max_vel~%std_msgs/Float32 max_acc~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Goals)))
  "Returns full string definition for message of type 'Goals"
  (cl:format cl:nil "std_msgs/Header header~%~%# Goal waypoints~%geometry_msgs/Pose[] waypoints~%geometry_msgs/Twist[] velocities~%geometry_msgs/Accel[] accelerations~%~%std_msgs/Bool[] velocities_mask  # 0 means use the value, 1 means ignore the value~%std_msgs/Bool[] accelerations_mask # 0 means use the value, 1 means ignore the value~%~%std_msgs/Float32 time_factor_terminal~%std_msgs/Float32 time_factor~%std_msgs/Float32 max_vel~%std_msgs/Float32 max_acc~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Accel~%# This expresses acceleration in free space broken into its linear and angular parts.~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: std_msgs/Bool~%bool data~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Goals>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'velocities) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'accelerations) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'velocities_mask) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'accelerations_mask) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'time_factor_terminal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'time_factor))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max_vel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max_acc))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Goals>))
  "Converts a ROS message object to a list"
  (cl:list 'Goals
    (cl:cons ':header (header msg))
    (cl:cons ':waypoints (waypoints msg))
    (cl:cons ':velocities (velocities msg))
    (cl:cons ':accelerations (accelerations msg))
    (cl:cons ':velocities_mask (velocities_mask msg))
    (cl:cons ':accelerations_mask (accelerations_mask msg))
    (cl:cons ':time_factor_terminal (time_factor_terminal msg))
    (cl:cons ':time_factor (time_factor msg))
    (cl:cons ':max_vel (max_vel msg))
    (cl:cons ':max_acc (max_acc msg))
))
