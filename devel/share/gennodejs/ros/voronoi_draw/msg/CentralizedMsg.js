// Auto-generated. Do not edit!

// (in-package voronoi_draw.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class CentralizedMsg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.valueArr = null;
    }
    else {
      if (initObj.hasOwnProperty('valueArr')) {
        this.valueArr = initObj.valueArr
      }
      else {
        this.valueArr = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CentralizedMsg
    // Serialize message field [valueArr]
    bufferOffset = _arraySerializer.float32(obj.valueArr, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CentralizedMsg
    let len;
    let data = new CentralizedMsg(null);
    // Deserialize message field [valueArr]
    data.valueArr = _arrayDeserializer.float32(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 4 * object.valueArr.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'voronoi_draw/CentralizedMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '06400f478916f41614d3caf5ce38a451';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[] valueArr
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CentralizedMsg(null);
    if (msg.valueArr !== undefined) {
      resolved.valueArr = msg.valueArr;
    }
    else {
      resolved.valueArr = []
    }

    return resolved;
    }
};

module.exports = CentralizedMsg;
