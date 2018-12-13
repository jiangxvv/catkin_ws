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

class course {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.angle = null;
      this.rate = null;
      this.acceleration = null;
    }
    else {
      if (initObj.hasOwnProperty('angle')) {
        this.angle = initObj.angle
      }
      else {
        this.angle = 0.0;
      }
      if (initObj.hasOwnProperty('rate')) {
        this.rate = initObj.rate
      }
      else {
        this.rate = 0.0;
      }
      if (initObj.hasOwnProperty('acceleration')) {
        this.acceleration = initObj.acceleration
      }
      else {
        this.acceleration = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type course
    // Serialize message field [angle]
    bufferOffset = _serializer.float64(obj.angle, buffer, bufferOffset);
    // Serialize message field [rate]
    bufferOffset = _serializer.float64(obj.rate, buffer, bufferOffset);
    // Serialize message field [acceleration]
    bufferOffset = _serializer.float64(obj.acceleration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type course
    let len;
    let data = new course(null);
    // Deserialize message field [angle]
    data.angle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [rate]
    data.rate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [acceleration]
    data.acceleration = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'usv_sim/course';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ae838d50ec8e47c9ce31d29911fbe48c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 angle
    float64 rate
    float64 acceleration
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new course(null);
    if (msg.angle !== undefined) {
      resolved.angle = msg.angle;
    }
    else {
      resolved.angle = 0.0
    }

    if (msg.rate !== undefined) {
      resolved.rate = msg.rate;
    }
    else {
      resolved.rate = 0.0
    }

    if (msg.acceleration !== undefined) {
      resolved.acceleration = msg.acceleration;
    }
    else {
      resolved.acceleration = 0.0
    }

    return resolved;
    }
};

module.exports = course;
