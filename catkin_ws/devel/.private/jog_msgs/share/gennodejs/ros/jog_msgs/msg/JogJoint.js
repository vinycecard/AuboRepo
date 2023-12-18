// Auto-generated. Do not edit!

// (in-package jog_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class JogJoint {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.joint_names = null;
      this.deltas = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('joint_names')) {
        this.joint_names = initObj.joint_names
      }
      else {
        this.joint_names = [];
      }
      if (initObj.hasOwnProperty('deltas')) {
        this.deltas = initObj.deltas
      }
      else {
        this.deltas = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JogJoint
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [joint_names]
    bufferOffset = _arraySerializer.string(obj.joint_names, buffer, bufferOffset, null);
    // Serialize message field [deltas]
    bufferOffset = _arraySerializer.float64(obj.deltas, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JogJoint
    let len;
    let data = new JogJoint(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_names]
    data.joint_names = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [deltas]
    data.deltas = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.joint_names.forEach((val) => {
      length += 4 + val.length;
    });
    length += 8 * object.deltas.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jog_msgs/JogJoint';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8d2aa14be64b51cf6374d198bfd489b2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This is a message to hold data to jog by specifying joint
    # displacement. You only need to set relative displacement to joint
    # angles (or displacements for linear joints).
    
    # header message. You must set frame_id to define the reference
    # coordinate system of the displacament
    Header header
    
    # Name list of the joints. You don't need to specify all joint of the
    # robot. Joint names are case-sensitive.
    string[] joint_names
    
    # Relative displacement of the joints to jog. The order must be
    # identical to joint_names. Unit is in radian for revolutive joints,
    # meter for linear joints.
    float64[] deltas
    
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
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JogJoint(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.joint_names !== undefined) {
      resolved.joint_names = msg.joint_names;
    }
    else {
      resolved.joint_names = []
    }

    if (msg.deltas !== undefined) {
      resolved.deltas = msg.deltas;
    }
    else {
      resolved.deltas = []
    }

    return resolved;
    }
};

module.exports = JogJoint;
