// Auto-generated. Do not edit!

// (in-package tip.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let UnicycleInfoStruct = require('./UnicycleInfoStruct.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class UnicycleInfoMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.packet = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('packet')) {
        this.packet = initObj.packet
      }
      else {
        this.packet = new UnicycleInfoStruct();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UnicycleInfoMsg
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [packet]
    bufferOffset = UnicycleInfoStruct.serialize(obj.packet, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UnicycleInfoMsg
    let len;
    let data = new UnicycleInfoMsg(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [packet]
    data.packet = UnicycleInfoStruct.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += UnicycleInfoStruct.getMessageSize(object.packet);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tip/UnicycleInfoMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '17316b31155b675ddbba08720370f6af';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    UnicycleInfoStruct packet
    
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
    MSG: tip/UnicycleInfoStruct
    Header header
    int32 TransmitterID
    float32 AgentPosX
    float32 AgentPosY
    float32 AgentTheta
    float32 VirtualCenterX
    float32 VirtualCenterY
    float32 V_BLF
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UnicycleInfoMsg(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.packet !== undefined) {
      resolved.packet = UnicycleInfoStruct.Resolve(msg.packet)
    }
    else {
      resolved.packet = new UnicycleInfoStruct()
    }

    return resolved;
    }
};

module.exports = UnicycleInfoMsg;
