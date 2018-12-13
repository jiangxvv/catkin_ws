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

class actuation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.tug1 = null;
      this.tug2 = null;
      this.tug3 = null;
      this.tug4 = null;
    }
    else {
      if (initObj.hasOwnProperty('tug1')) {
        this.tug1 = initObj.tug1
      }
      else {
        this.tug1 = 0.0;
      }
      if (initObj.hasOwnProperty('tug2')) {
        this.tug2 = initObj.tug2
      }
      else {
        this.tug2 = 0.0;
      }
      if (initObj.hasOwnProperty('tug3')) {
        this.tug3 = initObj.tug3
      }
      else {
        this.tug3 = 0.0;
      }
      if (initObj.hasOwnProperty('tug4')) {
        this.tug4 = initObj.tug4
      }
      else {
        this.tug4 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type actuation
    // Serialize message field [tug1]
    bufferOffset = _serializer.float64(obj.tug1, buffer, bufferOffset);
    // Serialize message field [tug2]
    bufferOffset = _serializer.float64(obj.tug2, buffer, bufferOffset);
    // Serialize message field [tug3]
    bufferOffset = _serializer.float64(obj.tug3, buffer, bufferOffset);
    // Serialize message field [tug4]
    bufferOffset = _serializer.float64(obj.tug4, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type actuation
    let len;
    let data = new actuation(null);
    // Deserialize message field [tug1]
    data.tug1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tug2]
    data.tug2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tug3]
    data.tug3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [tug4]
    data.tug4 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'usv_towing/actuation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6d5d273aeba236b992836cce53bb09da';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 tug1
    float64 tug2
    float64 tug3
    float64 tug4
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new actuation(null);
    if (msg.tug1 !== undefined) {
      resolved.tug1 = msg.tug1;
    }
    else {
      resolved.tug1 = 0.0
    }

    if (msg.tug2 !== undefined) {
      resolved.tug2 = msg.tug2;
    }
    else {
      resolved.tug2 = 0.0
    }

    if (msg.tug3 !== undefined) {
      resolved.tug3 = msg.tug3;
    }
    else {
      resolved.tug3 = 0.0
    }

    if (msg.tug4 !== undefined) {
      resolved.tug4 = msg.tug4;
    }
    else {
      resolved.tug4 = 0.0
    }

    return resolved;
    }
};

module.exports = actuation;
