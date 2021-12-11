// Auto-generated. Do not edit!

// (in-package tip.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class UnicycleInfoStruct {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.TransmitterID = null;
      this.AgentPosX = null;
      this.AgentPosY = null;
      this.AgentTheta = null;
      this.VirtualCenterX = null;
      this.VirtualCenterY = null;
      this.V_BLF = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('TransmitterID')) {
        this.TransmitterID = initObj.TransmitterID
      }
      else {
        this.TransmitterID = 0;
      }
      if (initObj.hasOwnProperty('AgentPosX')) {
        this.AgentPosX = initObj.AgentPosX
      }
      else {
        this.AgentPosX = 0.0;
      }
      if (initObj.hasOwnProperty('AgentPosY')) {
        this.AgentPosY = initObj.AgentPosY
      }
      else {
        this.AgentPosY = 0.0;
      }
      if (initObj.hasOwnProperty('AgentTheta')) {
        this.AgentTheta = initObj.AgentTheta
      }
      else {
        this.AgentTheta = 0.0;
      }
      if (initObj.hasOwnProperty('VirtualCenterX')) {
        this.VirtualCenterX = initObj.VirtualCenterX
      }
      else {
        this.VirtualCenterX = 0.0;
      }
      if (initObj.hasOwnProperty('VirtualCenterY')) {
        this.VirtualCenterY = initObj.VirtualCenterY
      }
      else {
        this.VirtualCenterY = 0.0;
      }
      if (initObj.hasOwnProperty('V_BLF')) {
        this.V_BLF = initObj.V_BLF
      }
      else {
        this.V_BLF = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UnicycleInfoStruct
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [TransmitterID]
    bufferOffset = _serializer.int32(obj.TransmitterID, buffer, bufferOffset);
    // Serialize message field [AgentPosX]
    bufferOffset = _serializer.float32(obj.AgentPosX, buffer, bufferOffset);
    // Serialize message field [AgentPosY]
    bufferOffset = _serializer.float32(obj.AgentPosY, buffer, bufferOffset);
    // Serialize message field [AgentTheta]
    bufferOffset = _serializer.float32(obj.AgentTheta, buffer, bufferOffset);
    // Serialize message field [VirtualCenterX]
    bufferOffset = _serializer.float32(obj.VirtualCenterX, buffer, bufferOffset);
    // Serialize message field [VirtualCenterY]
    bufferOffset = _serializer.float32(obj.VirtualCenterY, buffer, bufferOffset);
    // Serialize message field [V_BLF]
    bufferOffset = _serializer.float32(obj.V_BLF, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UnicycleInfoStruct
    let len;
    let data = new UnicycleInfoStruct(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [TransmitterID]
    data.TransmitterID = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [AgentPosX]
    data.AgentPosX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [AgentPosY]
    data.AgentPosY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [AgentTheta]
    data.AgentTheta = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [VirtualCenterX]
    data.VirtualCenterX = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [VirtualCenterY]
    data.VirtualCenterY = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [V_BLF]
    data.V_BLF = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'tip/UnicycleInfoStruct';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5cea3926dc7b0a30646b151951d6b5f5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    int32 TransmitterID
    float32 AgentPosX
    float32 AgentPosY
    float32 AgentTheta
    float32 VirtualCenterX
    float32 VirtualCenterY
    float32 V_BLF
    
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
    const resolved = new UnicycleInfoStruct(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.TransmitterID !== undefined) {
      resolved.TransmitterID = msg.TransmitterID;
    }
    else {
      resolved.TransmitterID = 0
    }

    if (msg.AgentPosX !== undefined) {
      resolved.AgentPosX = msg.AgentPosX;
    }
    else {
      resolved.AgentPosX = 0.0
    }

    if (msg.AgentPosY !== undefined) {
      resolved.AgentPosY = msg.AgentPosY;
    }
    else {
      resolved.AgentPosY = 0.0
    }

    if (msg.AgentTheta !== undefined) {
      resolved.AgentTheta = msg.AgentTheta;
    }
    else {
      resolved.AgentTheta = 0.0
    }

    if (msg.VirtualCenterX !== undefined) {
      resolved.VirtualCenterX = msg.VirtualCenterX;
    }
    else {
      resolved.VirtualCenterX = 0.0
    }

    if (msg.VirtualCenterY !== undefined) {
      resolved.VirtualCenterY = msg.VirtualCenterY;
    }
    else {
      resolved.VirtualCenterY = 0.0
    }

    if (msg.V_BLF !== undefined) {
      resolved.V_BLF = msg.V_BLF;
    }
    else {
      resolved.V_BLF = 0.0
    }

    return resolved;
    }
};

module.exports = UnicycleInfoStruct;
