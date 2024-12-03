;; Auto-generated. Do not edit!


(when (boundp 'gestelt_msgs::CommanderState)
  (if (not (find-package "GESTELT_MSGS"))
    (make-package "GESTELT_MSGS"))
  (shadow 'CommanderState (find-package "GESTELT_MSGS")))
(unless (find-package "GESTELT_MSGS::COMMANDERSTATE")
  (make-package "GESTELT_MSGS::COMMANDERSTATE"))

(in-package "ROS")
;;//! \htmlinclude CommanderState.msg.html


(defclass gestelt_msgs::CommanderState
  :super ros::object
  :slots (_drone_id _traj_server_state _planner_server_state _uav_state _armed ))

(defmethod gestelt_msgs::CommanderState
  (:init
   (&key
    ((:drone_id __drone_id) 0)
    ((:traj_server_state __traj_server_state) "")
    ((:planner_server_state __planner_server_state) "")
    ((:uav_state __uav_state) "")
    ((:armed __armed) nil)
    )
   (send-super :init)
   (setq _drone_id (round __drone_id))
   (setq _traj_server_state (string __traj_server_state))
   (setq _planner_server_state (string __planner_server_state))
   (setq _uav_state (string __uav_state))
   (setq _armed __armed)
   self)
  (:drone_id
   (&optional __drone_id)
   (if __drone_id (setq _drone_id __drone_id)) _drone_id)
  (:traj_server_state
   (&optional __traj_server_state)
   (if __traj_server_state (setq _traj_server_state __traj_server_state)) _traj_server_state)
  (:planner_server_state
   (&optional __planner_server_state)
   (if __planner_server_state (setq _planner_server_state __planner_server_state)) _planner_server_state)
  (:uav_state
   (&optional __uav_state)
   (if __uav_state (setq _uav_state __uav_state)) _uav_state)
  (:armed
   (&optional (__armed :null))
   (if (not (eq __armed :null)) (setq _armed __armed)) _armed)
  (:serialization-length
   ()
   (+
    ;; int32 _drone_id
    4
    ;; string _traj_server_state
    4 (length _traj_server_state)
    ;; string _planner_server_state
    4 (length _planner_server_state)
    ;; string _uav_state
    4 (length _uav_state)
    ;; bool _armed
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _drone_id
       (write-long _drone_id s)
     ;; string _traj_server_state
       (write-long (length _traj_server_state) s) (princ _traj_server_state s)
     ;; string _planner_server_state
       (write-long (length _planner_server_state) s) (princ _planner_server_state s)
     ;; string _uav_state
       (write-long (length _uav_state) s) (princ _uav_state s)
     ;; bool _armed
       (if _armed (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _drone_id
     (setq _drone_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _traj_server_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _traj_server_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _planner_server_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _planner_server_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _uav_state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _uav_state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _armed
     (setq _armed (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get gestelt_msgs::CommanderState :md5sum-) "b2808123e870beb3f216ec9c58781e6e")
(setf (get gestelt_msgs::CommanderState :datatype-) "gestelt_msgs/CommanderState")
(setf (get gestelt_msgs::CommanderState :definition-)
      "int32 drone_id
string traj_server_state
string planner_server_state
string uav_state
bool armed
")



(provide :gestelt_msgs/CommanderState "b2808123e870beb3f216ec9c58781e6e")


