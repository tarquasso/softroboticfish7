// Auto-generated. Do not edit!

// (in-package fish_msgs.msg)


"use strict";

let _serializer = require('../base_serialize.js');
let _deserializer = require('../base_deserialize.js');
let _finder = require('../find.js');

//-----------------------------------------------------------

class joystick_in {
  constructor() {
    this.freq_ctrl = 0;
    this.speed_ctrl = 0;
    this.depth_ctrl = 0;
    this.yaw_ctrl = 0;
  }

  static serialize(obj, bufferInfo) {
    // Serializes a message object of type joystick_in
    // Serialize message field [freq_ctrl]
    bufferInfo = _serializer.int8(obj.freq_ctrl, bufferInfo);
    // Serialize message field [speed_ctrl]
    bufferInfo = _serializer.int8(obj.speed_ctrl, bufferInfo);
    // Serialize message field [depth_ctrl]
    bufferInfo = _serializer.int8(obj.depth_ctrl, bufferInfo);
    // Serialize message field [yaw_ctrl]
    bufferInfo = _serializer.int8(obj.yaw_ctrl, bufferInfo);
    return bufferInfo;
  }

  static deserialize(buffer) {
    //deserializes a message object of type joystick_in
    let tmp;
    let len;
    let data = new joystick_in();
    // Deserialize message field [freq_ctrl]
    tmp = _deserializer.int8(buffer);
    data.freq_ctrl = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [speed_ctrl]
    tmp = _deserializer.int8(buffer);
    data.speed_ctrl = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [depth_ctrl]
    tmp = _deserializer.int8(buffer);
    data.depth_ctrl = tmp.data;
    buffer = tmp.buffer;
    // Deserialize message field [yaw_ctrl]
    tmp = _deserializer.int8(buffer);
    data.yaw_ctrl = tmp.data;
    buffer = tmp.buffer;
    return {
      data: data,
      buffer: buffer
    }
  }

  static datatype() {
    // Returns string type for a message object
    return 'fish_msgs/joystick_in';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0c650c89727301b1e2298a1c19175b51';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 freq_ctrl
    int8 speed_ctrl
    int8 depth_ctrl
    int8 yaw_ctrl
    
    `;
  }

};

module.exports = joystick_in;
