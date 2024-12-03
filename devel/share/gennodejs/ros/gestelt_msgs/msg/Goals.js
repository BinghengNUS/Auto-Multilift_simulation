// Auto-generated. Do not edit!

// (in-package gestelt_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Goals {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.waypoints = null;
      this.velocities = null;
      this.accelerations = null;
      this.velocities_mask = null;
      this.accelerations_mask = null;
      this.time_factor_terminal = null;
      this.time_factor = null;
      this.max_vel = null;
      this.max_acc = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('waypoints')) {
        this.waypoints = initObj.waypoints
      }
      else {
        this.waypoints = [];
      }
      if (initObj.hasOwnProperty('velocities')) {
        this.velocities = initObj.velocities
      }
      else {
        this.velocities = [];
      }
      if (initObj.hasOwnProperty('accelerations')) {
        this.accelerations = initObj.accelerations
      }
      else {
        this.accelerations = [];
      }
      if (initObj.hasOwnProperty('velocities_mask')) {
        this.velocities_mask = initObj.velocities_mask
      }
      else {
        this.velocities_mask = [];
      }
      if (initObj.hasOwnProperty('accelerations_mask')) {
        this.accelerations_mask = initObj.accelerations_mask
      }
      else {
        this.accelerations_mask = [];
      }
      if (initObj.hasOwnProperty('time_factor_terminal')) {
        this.time_factor_terminal = initObj.time_factor_terminal
      }
      else {
        this.time_factor_terminal = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('time_factor')) {
        this.time_factor = initObj.time_factor
      }
      else {
        this.time_factor = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('max_vel')) {
        this.max_vel = initObj.max_vel
      }
      else {
        this.max_vel = new std_msgs.msg.Float32();
      }
      if (initObj.hasOwnProperty('max_acc')) {
        this.max_acc = initObj.max_acc
      }
      else {
        this.max_acc = new std_msgs.msg.Float32();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Goals
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [waypoints]
    // Serialize the length for message field [waypoints]
    bufferOffset = _serializer.uint32(obj.waypoints.length, buffer, bufferOffset);
    obj.waypoints.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Pose.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [velocities]
    // Serialize the length for message field [velocities]
    bufferOffset = _serializer.uint32(obj.velocities.length, buffer, bufferOffset);
    obj.velocities.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Twist.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [accelerations]
    // Serialize the length for message field [accelerations]
    bufferOffset = _serializer.uint32(obj.accelerations.length, buffer, bufferOffset);
    obj.accelerations.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Accel.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [velocities_mask]
    // Serialize the length for message field [velocities_mask]
    bufferOffset = _serializer.uint32(obj.velocities_mask.length, buffer, bufferOffset);
    obj.velocities_mask.forEach((val) => {
      bufferOffset = std_msgs.msg.Bool.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [accelerations_mask]
    // Serialize the length for message field [accelerations_mask]
    bufferOffset = _serializer.uint32(obj.accelerations_mask.length, buffer, bufferOffset);
    obj.accelerations_mask.forEach((val) => {
      bufferOffset = std_msgs.msg.Bool.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [time_factor_terminal]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.time_factor_terminal, buffer, bufferOffset);
    // Serialize message field [time_factor]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.time_factor, buffer, bufferOffset);
    // Serialize message field [max_vel]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.max_vel, buffer, bufferOffset);
    // Serialize message field [max_acc]
    bufferOffset = std_msgs.msg.Float32.serialize(obj.max_acc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Goals
    let len;
    let data = new Goals(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [waypoints]
    // Deserialize array length for message field [waypoints]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.waypoints = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.waypoints[i] = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [velocities]
    // Deserialize array length for message field [velocities]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.velocities = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.velocities[i] = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [accelerations]
    // Deserialize array length for message field [accelerations]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.accelerations = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.accelerations[i] = geometry_msgs.msg.Accel.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [velocities_mask]
    // Deserialize array length for message field [velocities_mask]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.velocities_mask = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.velocities_mask[i] = std_msgs.msg.Bool.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [accelerations_mask]
    // Deserialize array length for message field [accelerations_mask]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.accelerations_mask = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.accelerations_mask[i] = std_msgs.msg.Bool.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [time_factor_terminal]
    data.time_factor_terminal = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [time_factor]
    data.time_factor = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_vel]
    data.max_vel = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    // Deserialize message field [max_acc]
    data.max_acc = std_msgs.msg.Float32.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 56 * object.waypoints.length;
    length += 48 * object.velocities.length;
    length += 48 * object.accelerations.length;
    length += object.velocities_mask.length;
    length += object.accelerations_mask.length;
    return length + 36;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gestelt_msgs/Goals';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '177ff0121ddb3a9dfb30d4b1b6d4fd32';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Goals(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.waypoints !== undefined) {
      resolved.waypoints = new Array(msg.waypoints.length);
      for (let i = 0; i < resolved.waypoints.length; ++i) {
        resolved.waypoints[i] = geometry_msgs.msg.Pose.Resolve(msg.waypoints[i]);
      }
    }
    else {
      resolved.waypoints = []
    }

    if (msg.velocities !== undefined) {
      resolved.velocities = new Array(msg.velocities.length);
      for (let i = 0; i < resolved.velocities.length; ++i) {
        resolved.velocities[i] = geometry_msgs.msg.Twist.Resolve(msg.velocities[i]);
      }
    }
    else {
      resolved.velocities = []
    }

    if (msg.accelerations !== undefined) {
      resolved.accelerations = new Array(msg.accelerations.length);
      for (let i = 0; i < resolved.accelerations.length; ++i) {
        resolved.accelerations[i] = geometry_msgs.msg.Accel.Resolve(msg.accelerations[i]);
      }
    }
    else {
      resolved.accelerations = []
    }

    if (msg.velocities_mask !== undefined) {
      resolved.velocities_mask = new Array(msg.velocities_mask.length);
      for (let i = 0; i < resolved.velocities_mask.length; ++i) {
        resolved.velocities_mask[i] = std_msgs.msg.Bool.Resolve(msg.velocities_mask[i]);
      }
    }
    else {
      resolved.velocities_mask = []
    }

    if (msg.accelerations_mask !== undefined) {
      resolved.accelerations_mask = new Array(msg.accelerations_mask.length);
      for (let i = 0; i < resolved.accelerations_mask.length; ++i) {
        resolved.accelerations_mask[i] = std_msgs.msg.Bool.Resolve(msg.accelerations_mask[i]);
      }
    }
    else {
      resolved.accelerations_mask = []
    }

    if (msg.time_factor_terminal !== undefined) {
      resolved.time_factor_terminal = std_msgs.msg.Float32.Resolve(msg.time_factor_terminal)
    }
    else {
      resolved.time_factor_terminal = new std_msgs.msg.Float32()
    }

    if (msg.time_factor !== undefined) {
      resolved.time_factor = std_msgs.msg.Float32.Resolve(msg.time_factor)
    }
    else {
      resolved.time_factor = new std_msgs.msg.Float32()
    }

    if (msg.max_vel !== undefined) {
      resolved.max_vel = std_msgs.msg.Float32.Resolve(msg.max_vel)
    }
    else {
      resolved.max_vel = new std_msgs.msg.Float32()
    }

    if (msg.max_acc !== undefined) {
      resolved.max_acc = std_msgs.msg.Float32.Resolve(msg.max_acc)
    }
    else {
      resolved.max_acc = new std_msgs.msg.Float32()
    }

    return resolved;
    }
};

module.exports = Goals;
