// Auto-generated. Do not edit!

// (in-package gnc_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class MotionStamped {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.time = null;
      this.pose = null;
      this.vel = null;
      this.accel = null;
    }
    else {
      if (initObj.hasOwnProperty('time')) {
        this.time = initObj.time
      }
      else {
        this.time = 0.0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('accel')) {
        this.accel = initObj.accel
      }
      else {
        this.accel = new geometry_msgs.msg.Vector3();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MotionStamped
    // Serialize message field [time]
    bufferOffset = _serializer.float32(obj.time, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [vel]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.vel, buffer, bufferOffset);
    // Serialize message field [accel]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.accel, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MotionStamped
    let len;
    let data = new MotionStamped(null);
    // Deserialize message field [time]
    data.time = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [vel]
    data.vel = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [accel]
    data.accel = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 76;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gnc_msgs/MotionStamped';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6633c63ac0d3fdc5b4d6fb7666e8181';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 time
    geometry_msgs/Vector3 pose
    geometry_msgs/Vector3  vel
    geometry_msgs/Vector3  accel
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new MotionStamped(null);
    if (msg.time !== undefined) {
      resolved.time = msg.time;
    }
    else {
      resolved.time = 0.0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Vector3.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Vector3()
    }

    if (msg.vel !== undefined) {
      resolved.vel = geometry_msgs.msg.Vector3.Resolve(msg.vel)
    }
    else {
      resolved.vel = new geometry_msgs.msg.Vector3()
    }

    if (msg.accel !== undefined) {
      resolved.accel = geometry_msgs.msg.Vector3.Resolve(msg.accel)
    }
    else {
      resolved.accel = new geometry_msgs.msg.Vector3()
    }

    return resolved;
    }
};

module.exports = MotionStamped;
