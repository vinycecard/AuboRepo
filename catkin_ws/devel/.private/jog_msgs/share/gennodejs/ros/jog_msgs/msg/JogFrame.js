// Auto-generated. Do not edit!

// (in-package jog_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class JogFrame {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.group_name = null;
      this.link_name = null;
      this.linear_delta = null;
      this.angular_delta = null;
      this.avoid_collisions = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('group_name')) {
        this.group_name = initObj.group_name
      }
      else {
        this.group_name = '';
      }
      if (initObj.hasOwnProperty('link_name')) {
        this.link_name = initObj.link_name
      }
      else {
        this.link_name = '';
      }
      if (initObj.hasOwnProperty('linear_delta')) {
        this.linear_delta = initObj.linear_delta
      }
      else {
        this.linear_delta = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('angular_delta')) {
        this.angular_delta = initObj.angular_delta
      }
      else {
        this.angular_delta = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('avoid_collisions')) {
        this.avoid_collisions = initObj.avoid_collisions
      }
      else {
        this.avoid_collisions = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JogFrame
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [group_name]
    bufferOffset = _serializer.string(obj.group_name, buffer, bufferOffset);
    // Serialize message field [link_name]
    bufferOffset = _serializer.string(obj.link_name, buffer, bufferOffset);
    // Serialize message field [linear_delta]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.linear_delta, buffer, bufferOffset);
    // Serialize message field [angular_delta]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.angular_delta, buffer, bufferOffset);
    // Serialize message field [avoid_collisions]
    bufferOffset = _serializer.bool(obj.avoid_collisions, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JogFrame
    let len;
    let data = new JogFrame(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [group_name]
    data.group_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [link_name]
    data.link_name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [linear_delta]
    data.linear_delta = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [angular_delta]
    data.angular_delta = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [avoid_collisions]
    data.avoid_collisions = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += object.group_name.length;
    length += object.link_name.length;
    return length + 57;
  }

  static datatype() {
    // Returns string type for a message object
    return 'jog_msgs/JogFrame';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e342f29bf6beaf00261bdae365abfff9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # This is a message to hold data to jog by specifying a target
    # frame. It uses MoveIt! kinematics, so you need to specify the
    # JointGroup name to use in group_name. (lienar|angular)_delta is the
    # amount of displacement.
    
    # header message. You must set frame_id to define the reference
    # coordinate system of the displacament
    Header header
    
    # Name of JointGroup of MoveIt!
    string group_name
    
    # Target link name to jog. The link must be in the JoingGroup
    string link_name
    
    # Linear displacement vector to jog. The refrence frame is defined by
    # frame_id in header. Unit is in meter.
    geometry_msgs/Vector3 linear_delta
    
    # Angular displacement vector to jog. The refrence frame is defined by
    # frame_id in header. Unit is in radian.
    geometry_msgs/Vector3 angular_delta
    
    # It uses avoid_collisions option of MoveIt! kinematics. If it is
    # true, the robot doesn't move if any collisions occured.
    bool avoid_collisions
    
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
    const resolved = new JogFrame(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.group_name !== undefined) {
      resolved.group_name = msg.group_name;
    }
    else {
      resolved.group_name = ''
    }

    if (msg.link_name !== undefined) {
      resolved.link_name = msg.link_name;
    }
    else {
      resolved.link_name = ''
    }

    if (msg.linear_delta !== undefined) {
      resolved.linear_delta = geometry_msgs.msg.Vector3.Resolve(msg.linear_delta)
    }
    else {
      resolved.linear_delta = new geometry_msgs.msg.Vector3()
    }

    if (msg.angular_delta !== undefined) {
      resolved.angular_delta = geometry_msgs.msg.Vector3.Resolve(msg.angular_delta)
    }
    else {
      resolved.angular_delta = new geometry_msgs.msg.Vector3()
    }

    if (msg.avoid_collisions !== undefined) {
      resolved.avoid_collisions = msg.avoid_collisions;
    }
    else {
      resolved.avoid_collisions = false
    }

    return resolved;
    }
};

module.exports = JogFrame;
