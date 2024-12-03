// Auto-generated. Do not edit!

// (in-package gestelt_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CommanderState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.drone_id = null;
      this.traj_server_state = null;
      this.planner_server_state = null;
      this.uav_state = null;
      this.armed = null;
    }
    else {
      if (initObj.hasOwnProperty('drone_id')) {
        this.drone_id = initObj.drone_id
      }
      else {
        this.drone_id = 0;
      }
      if (initObj.hasOwnProperty('traj_server_state')) {
        this.traj_server_state = initObj.traj_server_state
      }
      else {
        this.traj_server_state = '';
      }
      if (initObj.hasOwnProperty('planner_server_state')) {
        this.planner_server_state = initObj.planner_server_state
      }
      else {
        this.planner_server_state = '';
      }
      if (initObj.hasOwnProperty('uav_state')) {
        this.uav_state = initObj.uav_state
      }
      else {
        this.uav_state = '';
      }
      if (initObj.hasOwnProperty('armed')) {
        this.armed = initObj.armed
      }
      else {
        this.armed = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommanderState
    // Serialize message field [drone_id]
    bufferOffset = _serializer.int32(obj.drone_id, buffer, bufferOffset);
    // Serialize message field [traj_server_state]
    bufferOffset = _serializer.string(obj.traj_server_state, buffer, bufferOffset);
    // Serialize message field [planner_server_state]
    bufferOffset = _serializer.string(obj.planner_server_state, buffer, bufferOffset);
    // Serialize message field [uav_state]
    bufferOffset = _serializer.string(obj.uav_state, buffer, bufferOffset);
    // Serialize message field [armed]
    bufferOffset = _serializer.bool(obj.armed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommanderState
    let len;
    let data = new CommanderState(null);
    // Deserialize message field [drone_id]
    data.drone_id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [traj_server_state]
    data.traj_server_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [planner_server_state]
    data.planner_server_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [uav_state]
    data.uav_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [armed]
    data.armed = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.traj_server_state);
    length += _getByteLength(object.planner_server_state);
    length += _getByteLength(object.uav_state);
    return length + 17;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gestelt_msgs/CommanderState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b2808123e870beb3f216ec9c58781e6e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 drone_id
    string traj_server_state
    string planner_server_state
    string uav_state
    bool armed
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CommanderState(null);
    if (msg.drone_id !== undefined) {
      resolved.drone_id = msg.drone_id;
    }
    else {
      resolved.drone_id = 0
    }

    if (msg.traj_server_state !== undefined) {
      resolved.traj_server_state = msg.traj_server_state;
    }
    else {
      resolved.traj_server_state = ''
    }

    if (msg.planner_server_state !== undefined) {
      resolved.planner_server_state = msg.planner_server_state;
    }
    else {
      resolved.planner_server_state = ''
    }

    if (msg.uav_state !== undefined) {
      resolved.uav_state = msg.uav_state;
    }
    else {
      resolved.uav_state = ''
    }

    if (msg.armed !== undefined) {
      resolved.armed = msg.armed;
    }
    else {
      resolved.armed = false
    }

    return resolved;
    }
};

module.exports = CommanderState;
