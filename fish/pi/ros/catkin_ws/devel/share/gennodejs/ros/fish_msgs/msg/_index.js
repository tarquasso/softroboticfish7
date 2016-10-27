
"use strict";

let PumpTestMsg = require('./PumpTestMsg.js');
let FishCtrlMsg = require('./FishCtrlMsg.js');
let mbedStatusMsg = require('./mbedStatusMsg.js');
let DepthTestMsg = require('./DepthTestMsg.js');
let mbedPumpStatusMsg = require('./mbedPumpStatusMsg.js');
let MbedDataMsg = require('./MbedDataMsg.js');
let joystick_in = require('./joystick_in.js');

module.exports = {
  PumpTestMsg: PumpTestMsg,
  FishCtrlMsg: FishCtrlMsg,
  mbedStatusMsg: mbedStatusMsg,
  DepthTestMsg: DepthTestMsg,
  mbedPumpStatusMsg: mbedPumpStatusMsg,
  MbedDataMsg: MbedDataMsg,
  joystick_in: joystick_in,
};
