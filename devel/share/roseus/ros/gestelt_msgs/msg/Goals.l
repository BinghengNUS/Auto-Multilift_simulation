;; Auto-generated. Do not edit!


(when (boundp 'gestelt_msgs::Goals)
  (if (not (find-package "GESTELT_MSGS"))
    (make-package "GESTELT_MSGS"))
  (shadow 'Goals (find-package "GESTELT_MSGS")))
(unless (find-package "GESTELT_MSGS::GOALS")
  (make-package "GESTELT_MSGS::GOALS"))

(in-package "ROS")
;;//! \htmlinclude Goals.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass gestelt_msgs::Goals
  :super ros::object
  :slots (_header _waypoints _velocities _accelerations _velocities_mask _accelerations_mask _time_factor_terminal _time_factor _max_vel _max_acc ))

(defmethod gestelt_msgs::Goals
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:waypoints __waypoints) ())
    ((:velocities __velocities) ())
    ((:accelerations __accelerations) ())
    ((:velocities_mask __velocities_mask) ())
    ((:accelerations_mask __accelerations_mask) ())
    ((:time_factor_terminal __time_factor_terminal) (instance std_msgs::Float32 :init))
    ((:time_factor __time_factor) (instance std_msgs::Float32 :init))
    ((:max_vel __max_vel) (instance std_msgs::Float32 :init))
    ((:max_acc __max_acc) (instance std_msgs::Float32 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _waypoints __waypoints)
   (setq _velocities __velocities)
   (setq _accelerations __accelerations)
   (setq _velocities_mask __velocities_mask)
   (setq _accelerations_mask __accelerations_mask)
   (setq _time_factor_terminal __time_factor_terminal)
   (setq _time_factor __time_factor)
   (setq _max_vel __max_vel)
   (setq _max_acc __max_acc)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:velocities
   (&rest __velocities)
   (if (keywordp (car __velocities))
       (send* _velocities __velocities)
     (progn
       (if __velocities (setq _velocities (car __velocities)))
       _velocities)))
  (:accelerations
   (&rest __accelerations)
   (if (keywordp (car __accelerations))
       (send* _accelerations __accelerations)
     (progn
       (if __accelerations (setq _accelerations (car __accelerations)))
       _accelerations)))
  (:velocities_mask
   (&rest __velocities_mask)
   (if (keywordp (car __velocities_mask))
       (send* _velocities_mask __velocities_mask)
     (progn
       (if __velocities_mask (setq _velocities_mask (car __velocities_mask)))
       _velocities_mask)))
  (:accelerations_mask
   (&rest __accelerations_mask)
   (if (keywordp (car __accelerations_mask))
       (send* _accelerations_mask __accelerations_mask)
     (progn
       (if __accelerations_mask (setq _accelerations_mask (car __accelerations_mask)))
       _accelerations_mask)))
  (:time_factor_terminal
   (&rest __time_factor_terminal)
   (if (keywordp (car __time_factor_terminal))
       (send* _time_factor_terminal __time_factor_terminal)
     (progn
       (if __time_factor_terminal (setq _time_factor_terminal (car __time_factor_terminal)))
       _time_factor_terminal)))
  (:time_factor
   (&rest __time_factor)
   (if (keywordp (car __time_factor))
       (send* _time_factor __time_factor)
     (progn
       (if __time_factor (setq _time_factor (car __time_factor)))
       _time_factor)))
  (:max_vel
   (&rest __max_vel)
   (if (keywordp (car __max_vel))
       (send* _max_vel __max_vel)
     (progn
       (if __max_vel (setq _max_vel (car __max_vel)))
       _max_vel)))
  (:max_acc
   (&rest __max_acc)
   (if (keywordp (car __max_acc))
       (send* _max_acc __max_acc)
     (progn
       (if __max_acc (setq _max_acc (car __max_acc)))
       _max_acc)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Pose[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ;; geometry_msgs/Twist[] _velocities
    (apply #'+ (send-all _velocities :serialization-length)) 4
    ;; geometry_msgs/Accel[] _accelerations
    (apply #'+ (send-all _accelerations :serialization-length)) 4
    ;; std_msgs/Bool[] _velocities_mask
    (apply #'+ (send-all _velocities_mask :serialization-length)) 4
    ;; std_msgs/Bool[] _accelerations_mask
    (apply #'+ (send-all _accelerations_mask :serialization-length)) 4
    ;; std_msgs/Float32 _time_factor_terminal
    (send _time_factor_terminal :serialization-length)
    ;; std_msgs/Float32 _time_factor
    (send _time_factor :serialization-length)
    ;; std_msgs/Float32 _max_vel
    (send _max_vel :serialization-length)
    ;; std_msgs/Float32 _max_acc
    (send _max_acc :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Pose[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Twist[] _velocities
     (write-long (length _velocities) s)
     (dolist (elem _velocities)
       (send elem :serialize s)
       )
     ;; geometry_msgs/Accel[] _accelerations
     (write-long (length _accelerations) s)
     (dolist (elem _accelerations)
       (send elem :serialize s)
       )
     ;; std_msgs/Bool[] _velocities_mask
     (write-long (length _velocities_mask) s)
     (dolist (elem _velocities_mask)
       (send elem :serialize s)
       )
     ;; std_msgs/Bool[] _accelerations_mask
     (write-long (length _accelerations_mask) s)
     (dolist (elem _accelerations_mask)
       (send elem :serialize s)
       )
     ;; std_msgs/Float32 _time_factor_terminal
       (send _time_factor_terminal :serialize s)
     ;; std_msgs/Float32 _time_factor
       (send _time_factor :serialize s)
     ;; std_msgs/Float32 _max_vel
       (send _max_vel :serialize s)
     ;; std_msgs/Float32 _max_acc
       (send _max_acc :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Pose[] _waypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints (let (r) (dotimes (i n) (push (instance geometry_msgs::Pose :init) r)) r))
     (dolist (elem- _waypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Twist[] _velocities
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities (let (r) (dotimes (i n) (push (instance geometry_msgs::Twist :init) r)) r))
     (dolist (elem- _velocities)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; geometry_msgs/Accel[] _accelerations
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _accelerations (let (r) (dotimes (i n) (push (instance geometry_msgs::Accel :init) r)) r))
     (dolist (elem- _accelerations)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Bool[] _velocities_mask
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _velocities_mask (let (r) (dotimes (i n) (push (instance std_msgs::Bool :init) r)) r))
     (dolist (elem- _velocities_mask)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Bool[] _accelerations_mask
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _accelerations_mask (let (r) (dotimes (i n) (push (instance std_msgs::Bool :init) r)) r))
     (dolist (elem- _accelerations_mask)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; std_msgs/Float32 _time_factor_terminal
     (send _time_factor_terminal :deserialize buf ptr-) (incf ptr- (send _time_factor_terminal :serialization-length))
   ;; std_msgs/Float32 _time_factor
     (send _time_factor :deserialize buf ptr-) (incf ptr- (send _time_factor :serialization-length))
   ;; std_msgs/Float32 _max_vel
     (send _max_vel :deserialize buf ptr-) (incf ptr- (send _max_vel :serialization-length))
   ;; std_msgs/Float32 _max_acc
     (send _max_acc :deserialize buf ptr-) (incf ptr- (send _max_acc :serialization-length))
   ;;
   self)
  )

(setf (get gestelt_msgs::Goals :md5sum-) "177ff0121ddb3a9dfb30d4b1b6d4fd32")
(setf (get gestelt_msgs::Goals :datatype-) "gestelt_msgs/Goals")
(setf (get gestelt_msgs::Goals :definition-)
      "std_msgs/Header header

# Goal waypoints
geometry_msgs/Pose[] waypoints
geometry_msgs/Twist[] velocities
geometry_msgs/Accel[] accelerations

std_msgs/Bool[] velocities_mask  # 0 means use the value, 1 means ignore the value
std_msgs/Bool[] accelerations_mask # 0 means use the value, 1 means ignore the value

std_msgs/Float32 time_factor_terminal
std_msgs/Float32 time_factor
std_msgs/Float32 max_vel
std_msgs/Float32 max_acc
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

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Twist
# This expresses velocity in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Accel
# This expresses acceleration in free space broken into its linear and angular parts.
Vector3  linear
Vector3  angular

================================================================================
MSG: std_msgs/Bool
bool data
================================================================================
MSG: std_msgs/Float32
float32 data
")



(provide :gestelt_msgs/Goals "177ff0121ddb3a9dfb30d4b1b6d4fd32")


