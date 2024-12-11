// Auto-generated. Do not edit!

// (in-package internal.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class BoardMove {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.start_x = null;
      this.start_y = null;
      this.end_x = null;
      this.end_y = null;
    }
    else {
      if (initObj.hasOwnProperty('start_x')) {
        this.start_x = initObj.start_x
      }
      else {
        this.start_x = 0;
      }
      if (initObj.hasOwnProperty('start_y')) {
        this.start_y = initObj.start_y
      }
      else {
        this.start_y = 0;
      }
      if (initObj.hasOwnProperty('end_x')) {
        this.end_x = initObj.end_x
      }
      else {
        this.end_x = 0;
      }
      if (initObj.hasOwnProperty('end_y')) {
        this.end_y = initObj.end_y
      }
      else {
        this.end_y = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BoardMove
    // Serialize message field [start_x]
    bufferOffset = _serializer.int32(obj.start_x, buffer, bufferOffset);
    // Serialize message field [start_y]
    bufferOffset = _serializer.int32(obj.start_y, buffer, bufferOffset);
    // Serialize message field [end_x]
    bufferOffset = _serializer.int32(obj.end_x, buffer, bufferOffset);
    // Serialize message field [end_y]
    bufferOffset = _serializer.int32(obj.end_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BoardMove
    let len;
    let data = new BoardMove(null);
    // Deserialize message field [start_x]
    data.start_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [start_y]
    data.start_y = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [end_x]
    data.end_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [end_y]
    data.end_y = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'internal/BoardMove';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2d9a81541f0e7558640a92686275d893';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 start_x
    int32 start_y
    int32 end_x
    int32 end_y
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BoardMove(null);
    if (msg.start_x !== undefined) {
      resolved.start_x = msg.start_x;
    }
    else {
      resolved.start_x = 0
    }

    if (msg.start_y !== undefined) {
      resolved.start_y = msg.start_y;
    }
    else {
      resolved.start_y = 0
    }

    if (msg.end_x !== undefined) {
      resolved.end_x = msg.end_x;
    }
    else {
      resolved.end_x = 0
    }

    if (msg.end_y !== undefined) {
      resolved.end_y = msg.end_y;
    }
    else {
      resolved.end_y = 0
    }

    return resolved;
    }
};

module.exports = BoardMove;
