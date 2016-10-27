// Auto-generated. Do not edit!

// (in-package fish_msgs.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class PumpTestMsg {
  constructor() {
    this.mode = 0;
    this.freq = 0.0;
    this.yaw = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type PumpTestMsg
    // Serialize message field [mode]
    bufferInfo = _serializer.int8(obj.mode, bufferInfo);
    // Serialize message field [freq]
    bufferInfo = _serializer.float32(obj.freq, bufferInfo);
    // Serialize message field [yaw]
    bufferInfo = _serializer.float32(obj.yaw, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type PumpTestMsg
    let tmp;
    let len;
    let data = new PumpTestMsg();
    // Deserialize message field [mode]
    tmp = _deserializer.int8(buffer);
    data.mode = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [freq]
    tmp = _deserializer.float32(buffer);
    data.freq = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [yaw]
    tmp = _deserializer.float32(buffer);
    data.yaw = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'fish_msgs/PumpTestMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '34fadd9771870a86de960e911d1d8c17';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 mode
    float32 freq
    float32 yaw
    
    `;
  }

};

module.exports = PumpTestMsg;
