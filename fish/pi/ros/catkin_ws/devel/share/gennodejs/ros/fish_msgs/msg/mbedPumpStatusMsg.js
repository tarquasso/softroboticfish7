// Auto-generated. Do not edit!

// (in-package fish_msgs.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class mbedPumpStatusMsg {
  constructor() {
    this.mode = 0;
    this.yaw = 0.0;
    this.freq = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type mbedPumpStatusMsg
    // Serialize message field [mode]
    bufferInfo = _serializer.int8(obj.mode, bufferInfo);
    // Serialize message field [yaw]
    bufferInfo = _serializer.float32(obj.yaw, bufferInfo);
    // Serialize message field [freq]
    bufferInfo = _serializer.float32(obj.freq, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type mbedPumpStatusMsg
    let tmp;
    let len;
    let data = new mbedPumpStatusMsg();
    // Deserialize message field [mode]
    tmp = _deserializer.int8(buffer);
    data.mode = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [yaw]
    tmp = _deserializer.float32(buffer);
    data.yaw = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [freq]
    tmp = _deserializer.float32(buffer);
    data.freq = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'fish_msgs/mbedPumpStatusMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd362e13e6bb848f15ea16da4058b765d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 mode
    float32 yaw
    float32 freq
    
    `;
  }

};

module.exports = mbedPumpStatusMsg;
