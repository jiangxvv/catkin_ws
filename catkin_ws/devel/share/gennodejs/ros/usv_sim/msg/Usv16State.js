// Auto-generated. Do not edit!

// (in-package usv_sim.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class Usv16State {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.t = null;
      this.pose = null;
      this.vel = null;
      this.acc = null;
    }
    else {
      if (initObj.hasOwnProperty('t')) {
        this.t = initObj.t
      }
      else {
        this.t = 0.0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose2D();
      }
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = new geometry_msgs.msg.Twist();
      }
      if (initObj.hasOwnProperty('acc')) {
        this.acc = initObj.acc
      }
      else {
        this.acc = new geometry_msgs.msg.Twist();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Usv16State
    // Serialize message field [t]
    bufferOffset = _serializer.float64(obj.t, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose2D.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [vel]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.vel, buffer, bufferOffset);
    // Serialize message field [acc]
    bufferOffset = geometry_msgs.msg.Twist.serialize(obj.acc, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Usv16State
    let len;
    let data = new Usv16State(null);
    // Deserialize message field [t]
    data.t = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose2D.deserialize(buffer, bufferOffset);
    // Deserialize message field [vel]
    data.vel = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    // Deserialize message field [acc]
    data.acc = geometry_msgs.msg.Twist.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 128;
  }

  static datatype() {
    // Returns string type for a message object
    return 'usv_sim/Usv16State';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '267dc2162d9371f64d61a71605f7ca88';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 t
    geometry_msgs/Pose2D  pose
    geometry_msgs/Twist  vel
    geometry_msgs/Twist  acc
    
    ================================================================================
    MSG: geometry_msgs/Pose2D
    # This expresses a position and orientation on a 2D manifold.
    
    float64 x
    float64 y
    float64 theta
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
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Usv16State(null);
    if (msg.t !== undefined) {
      resolved.t = msg.t;
    }
    else {
      resolved.t = 0.0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose2D.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose2D()
    }

    if (msg.vel !== undefined) {
      resolved.vel = geometry_msgs.msg.Twist.Resolve(msg.vel)
    }
    else {
      resolved.vel = new geometry_msgs.msg.Twist()
    }

    if (msg.acc !== undefined) {
      resolved.acc = geometry_msgs.msg.Twist.Resolve(msg.acc)
    }
    else {
      resolved.acc = new geometry_msgs.msg.Twist()
    }

    return resolved;
    }
};

module.exports = Usv16State;
