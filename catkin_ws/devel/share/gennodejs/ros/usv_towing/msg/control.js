// Auto-generated. Do not edit!

// (in-package usv_towing.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class control {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.surge = null;
      this.sway = null;
      this.yaw = null;
    }
    else {
      if (initObj.hasOwnProperty('surge')) {
        this.surge = initObj.surge
      }
      else {
        this.surge = 0.0;
      }
      if (initObj.hasOwnProperty('sway')) {
        this.sway = initObj.sway
      }
      else {
        this.sway = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type control
    // Serialize message field [surge]
    bufferOffset = _serializer.float64(obj.surge, buffer, bufferOffset);
    // Serialize message field [sway]
    bufferOffset = _serializer.float64(obj.sway, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type control
    let len;
    let data = new control(null);
    // Deserialize message field [surge]
    data.surge = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [sway]
    data.sway = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'usv_towing/control';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd86ccd8ad254d064df2af607e63b6ac0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 surge
    float64 sway
    float64 yaw
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new control(null);
    if (msg.surge !== undefined) {
      resolved.surge = msg.surge;
    }
    else {
      resolved.surge = 0.0
    }

    if (msg.sway !== undefined) {
      resolved.sway = msg.sway;
    }
    else {
      resolved.sway = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    return resolved;
    }
};

module.exports = control;
