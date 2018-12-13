// Auto-generated. Do not edit!

// (in-package usv_sim.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class actuator {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.T_p = null;
      this.Alpha_p = null;
      this.T_s = null;
      this.Alpha_s = null;
    }
    else {
      if (initObj.hasOwnProperty('T_p')) {
        this.T_p = initObj.T_p
      }
      else {
        this.T_p = 0.0;
      }
      if (initObj.hasOwnProperty('Alpha_p')) {
        this.Alpha_p = initObj.Alpha_p
      }
      else {
        this.Alpha_p = 0.0;
      }
      if (initObj.hasOwnProperty('T_s')) {
        this.T_s = initObj.T_s
      }
      else {
        this.T_s = 0.0;
      }
      if (initObj.hasOwnProperty('Alpha_s')) {
        this.Alpha_s = initObj.Alpha_s
      }
      else {
        this.Alpha_s = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type actuator
    // Serialize message field [T_p]
    bufferOffset = _serializer.float64(obj.T_p, buffer, bufferOffset);
    // Serialize message field [Alpha_p]
    bufferOffset = _serializer.float64(obj.Alpha_p, buffer, bufferOffset);
    // Serialize message field [T_s]
    bufferOffset = _serializer.float64(obj.T_s, buffer, bufferOffset);
    // Serialize message field [Alpha_s]
    bufferOffset = _serializer.float64(obj.Alpha_s, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type actuator
    let len;
    let data = new actuator(null);
    // Deserialize message field [T_p]
    data.T_p = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Alpha_p]
    data.Alpha_p = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [T_s]
    data.T_s = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Alpha_s]
    data.Alpha_s = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'usv_sim/actuator';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '058476a8fb2069bcb447843b2fbad502';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 T_p
    float64 Alpha_p
    float64 T_s
    float64 Alpha_s
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new actuator(null);
    if (msg.T_p !== undefined) {
      resolved.T_p = msg.T_p;
    }
    else {
      resolved.T_p = 0.0
    }

    if (msg.Alpha_p !== undefined) {
      resolved.Alpha_p = msg.Alpha_p;
    }
    else {
      resolved.Alpha_p = 0.0
    }

    if (msg.T_s !== undefined) {
      resolved.T_s = msg.T_s;
    }
    else {
      resolved.T_s = 0.0
    }

    if (msg.Alpha_s !== undefined) {
      resolved.Alpha_s = msg.Alpha_s;
    }
    else {
      resolved.Alpha_s = 0.0
    }

    return resolved;
    }
};

module.exports = actuator;
