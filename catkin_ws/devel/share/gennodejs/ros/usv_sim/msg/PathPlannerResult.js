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

class PathPlannerResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.wpt_x = null;
      this.wpt_y = null;
      this.wpt_psi = null;
    }
    else {
      if (initObj.hasOwnProperty('wpt_x')) {
        this.wpt_x = initObj.wpt_x
      }
      else {
        this.wpt_x = [];
      }
      if (initObj.hasOwnProperty('wpt_y')) {
        this.wpt_y = initObj.wpt_y
      }
      else {
        this.wpt_y = [];
      }
      if (initObj.hasOwnProperty('wpt_psi')) {
        this.wpt_psi = initObj.wpt_psi
      }
      else {
        this.wpt_psi = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PathPlannerResult
    // Serialize message field [wpt_x]
    bufferOffset = _arraySerializer.float64(obj.wpt_x, buffer, bufferOffset, null);
    // Serialize message field [wpt_y]
    bufferOffset = _arraySerializer.float64(obj.wpt_y, buffer, bufferOffset, null);
    // Serialize message field [wpt_psi]
    bufferOffset = _arraySerializer.float64(obj.wpt_psi, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PathPlannerResult
    let len;
    let data = new PathPlannerResult(null);
    // Deserialize message field [wpt_x]
    data.wpt_x = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [wpt_y]
    data.wpt_y = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [wpt_psi]
    data.wpt_psi = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.wpt_x.length;
    length += 8 * object.wpt_y.length;
    length += 8 * object.wpt_psi.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'usv_sim/PathPlannerResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6a6cc66078fa7c117d6ab9ada7708794';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    # result definition
    float64[] wpt_x
    float64[] wpt_y
    float64[] wpt_psi
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PathPlannerResult(null);
    if (msg.wpt_x !== undefined) {
      resolved.wpt_x = msg.wpt_x;
    }
    else {
      resolved.wpt_x = []
    }

    if (msg.wpt_y !== undefined) {
      resolved.wpt_y = msg.wpt_y;
    }
    else {
      resolved.wpt_y = []
    }

    if (msg.wpt_psi !== undefined) {
      resolved.wpt_psi = msg.wpt_psi;
    }
    else {
      resolved.wpt_psi = []
    }

    return resolved;
    }
};

module.exports = PathPlannerResult;
