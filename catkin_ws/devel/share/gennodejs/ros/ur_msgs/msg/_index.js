
"use strict";

let MasterboardDataMsg = require('./MasterboardDataMsg.js');
let RobotModeDataMsg = require('./RobotModeDataMsg.js');
let Digital = require('./Digital.js');
let ToolDataMsg = require('./ToolDataMsg.js');
let IOStates = require('./IOStates.js');
let RobotStateRTMsg = require('./RobotStateRTMsg.js');
let Analog = require('./Analog.js');

module.exports = {
  MasterboardDataMsg: MasterboardDataMsg,
  RobotModeDataMsg: RobotModeDataMsg,
  Digital: Digital,
  ToolDataMsg: ToolDataMsg,
  IOStates: IOStates,
  RobotStateRTMsg: RobotStateRTMsg,
  Analog: Analog,
};
