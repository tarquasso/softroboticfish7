// Auto-generated. Do not edit!

// (in-package fish_msgs.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class mbedStatusMsg {
  constructor() {
    this.mode = 0;
    this.value = 0.0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type mbedStatusMsg
    // Serialize message field [mode]
    bufferInfo = _serializer.int8(obj.mode, bufferInfo);
    // Serialize message field [value]
    bufferInfo = _serializer.float32(obj.value, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type mbedStatusMsg
    let tmp;
    let len;
    let data = new mbedStatusMsg();
    // Deserialize message field [mode]
    tmp = _deserializer.int8(buffer);
    data.mode = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [value]
    tmp = _deserializer.float32(buffer);
    data.value = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'fish_msgs/mbedStatusMsg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '392256b70b5280d59063ab9c2ddc1efa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 mode
    float32 value
    
    `;
  }

};

module.exports = mbedStatusMsg;
