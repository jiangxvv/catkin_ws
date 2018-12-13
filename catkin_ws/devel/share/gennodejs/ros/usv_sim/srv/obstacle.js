// Auto-generated. Do not edit!

// (in-package usv_sim.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class obstacleRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x_coor = null;
      this.y_coor = null;
      this.radius = null;
    }
    else {
      if (initObj.hasOwnProperty('x_coor')) {
        this.x_coor = initObj.x_coor
      }
      else {
        this.x_coor = 0.0;
      }
      if (initObj.hasOwnProperty('y_coor')) {
        this.y_coor = initObj.y_coor
      }
      else {
        this.y_coor = 0.0;
      }
      if (initObj.hasOwnProperty('radius')) {
        this.radius = initObj.radius
      }
      else {
        this.radius = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type obstacleRequest
    // Serialize message field [x_coor]
    bufferOffset = _serializer.float64(obj.x_coor, buffer, bufferOffset);
    // Serialize message field [y_coor]
    bufferOffset = _serializer.float64(obj.y_coor, buffer, bufferOffset);
    // Serialize message field [radius]
    bufferOffset = _serializer.float64(obj.radius, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type obstacleRequest
    let len;
    let data = new obstacleRequest(null);
    // Deserialize message field [x_coor]
    data.x_coor = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y_coor]
    data.y_coor = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [radius]
    data.radius = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'usv_sim/obstacleRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0a876fe65406998e7960f3ccc4c14ca1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x_coor
    float64 y_coor
    float64 radius
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new obstacleRequest(null);
    if (msg.x_coor !== undefined) {
      resolved.x_coor = msg.x_coor;
    }
    else {
      resolved.x_coor = 0.0
    }

    if (msg.y_coor !== undefined) {
      resolved.y_coor = msg.y_coor;
    }
    else {
      resolved.y_coor = 0.0
    }

    if (msg.radius !== undefined) {
      resolved.radius = msg.radius;
    }
    else {
      resolved.radius = 0.0
    }

    return resolved;
    }
};

class obstacleResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.response = null;
    }
    else {
      if (initObj.hasOwnProperty('response')) {
        this.response = initObj.response
      }
      else {
        this.response = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type obstacleResponse
    // Serialize message field [response]
    bufferOffset = _serializer.string(obj.response, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type obstacleResponse
    let len;
    let data = new obstacleResponse(null);
    // Deserialize message field [response]
    data.response = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.response.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'usv_sim/obstacleResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6de314e2dc76fbff2b6244a6ad70b68d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string response
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new obstacleResponse(null);
    if (msg.response !== undefined) {
      resolved.response = msg.response;
    }
    else {
      resolved.response = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: obstacleRequest,
  Response: obstacleResponse,
  md5sum() { return '11fe2b9b051393d66ed5d61bfe2c5ce7'; },
  datatype() { return 'usv_sim/obstacle'; }
};
