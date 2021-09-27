
"use strict";

let GetRobotMode = require('./GetRobotMode.js')
let IsProgramRunning = require('./IsProgramRunning.js')
let AddToLog = require('./AddToLog.js')
let IsProgramSaved = require('./IsProgramSaved.js')
let GetLoadedProgram = require('./GetLoadedProgram.js')
let Popup = require('./Popup.js')
let RawRequest = require('./RawRequest.js')
let GetSafetyMode = require('./GetSafetyMode.js')
let GetProgramState = require('./GetProgramState.js')
let Load = require('./Load.js')

module.exports = {
  GetRobotMode: GetRobotMode,
  IsProgramRunning: IsProgramRunning,
  AddToLog: AddToLog,
  IsProgramSaved: IsProgramSaved,
  GetLoadedProgram: GetLoadedProgram,
  Popup: Popup,
  RawRequest: RawRequest,
  GetSafetyMode: GetSafetyMode,
  GetProgramState: GetProgramState,
  Load: Load,
};
