// Auto-generated. Do not edit!

// (in-package fish_msgs.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class MbedDataMsg {
  constructor() {
    this.mode = 0;
    this.thrust = 0.0;
    this.dvalue = 0.0;
    this.yaw = 0.0;
    this.valve = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type MbedDataMsg
    // Serialize message field [mode]
    bufferInfo = _serializer.int8(obj.mode, bufferInfo);
    // Serialize message field [thrust]
    bufferInfo = _serializer.float32(obj.thrust, bufferInfo);
    // Serialize message field [dvalue]
    bufferInfo = _serializer.float32(obj.dvalue, bufferInfo);
    // Serialize message field [yaw]
    bufferInfo = _serializer.float32(obj.yaw, bufferInfo);
    // Serialize message field [valve]
    bufferInfo = _serializer.float32(obj.valve, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type MbedDataMsg
    let tmp;
    let len;
    let data = new MbedDataMsg();
    // Deserialize message field [mode]
    tmp = _deserializer.int8(buffer);
    data.mode = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [thrust]
    tmp = _deserializer.float32(buffer);
    data.thrust = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [dvalue]
    tmp = _deserializer.float32(buffer);
    data.dvalue = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [yaw]
    tmp = _deserializer.float32(buffer);
    data.yaw = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [valve]
    tmp = _deserializer.float32(buffer);
    data.valve = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'fish_msgs/MbedDataMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9305e87d9033eb2ef9f1c3927209a31b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 mode
    float32 thrust
    float32 dvalue
    float32 yaw
    float32 valve
    
    `;
  }

};

module.exports = MbedDataMsg;
