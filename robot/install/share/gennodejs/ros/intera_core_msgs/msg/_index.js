
"use strict";

let AnalogIOState = require('./AnalogIOState.js');
let IOComponentStatus = require('./IOComponentStatus.js');
let RobotAssemblyState = require('./RobotAssemblyState.js');
let IOStatus = require('./IOStatus.js');
let NavigatorStates = require('./NavigatorStates.js');
let DigitalIOStates = require('./DigitalIOStates.js');
let IODeviceConfiguration = require('./IODeviceConfiguration.js');
let CollisionAvoidanceState = require('./CollisionAvoidanceState.js');
let CameraControl = require('./CameraControl.js');
let EndpointState = require('./EndpointState.js');
let IONodeStatus = require('./IONodeStatus.js');
let CollisionDetectionState = require('./CollisionDetectionState.js');
let EndpointNamesArray = require('./EndpointNamesArray.js');
let HomingState = require('./HomingState.js');
let IOComponentConfiguration = require('./IOComponentConfiguration.js');
let IODeviceStatus = require('./IODeviceStatus.js');
let DigitalIOState = require('./DigitalIOState.js');
let JointCommand = require('./JointCommand.js');
let URDFConfiguration = require('./URDFConfiguration.js');
let EndpointStates = require('./EndpointStates.js');
let NavigatorState = require('./NavigatorState.js');
let IONodeConfiguration = require('./IONodeConfiguration.js');
let IOComponentCommand = require('./IOComponentCommand.js');
let AnalogIOStates = require('./AnalogIOStates.js');
let JointLimits = require('./JointLimits.js');
let CameraSettings = require('./CameraSettings.js');
let HeadPanCommand = require('./HeadPanCommand.js');
let InteractionControlState = require('./InteractionControlState.js');
let InteractionControlCommand = require('./InteractionControlCommand.js');
let SEAJointState = require('./SEAJointState.js');
let IODataStatus = require('./IODataStatus.js');
let DigitalOutputCommand = require('./DigitalOutputCommand.js');
let HeadState = require('./HeadState.js');
let AnalogOutputCommand = require('./AnalogOutputCommand.js');
let HomingCommand = require('./HomingCommand.js');
let CalibrationCommandActionFeedback = require('./CalibrationCommandActionFeedback.js');
let CalibrationCommandActionGoal = require('./CalibrationCommandActionGoal.js');
let CalibrationCommandResult = require('./CalibrationCommandResult.js');
let CalibrationCommandActionResult = require('./CalibrationCommandActionResult.js');
let CalibrationCommandGoal = require('./CalibrationCommandGoal.js');
let CalibrationCommandAction = require('./CalibrationCommandAction.js');
let CalibrationCommandFeedback = require('./CalibrationCommandFeedback.js');

module.exports = {
  AnalogIOState: AnalogIOState,
  IOComponentStatus: IOComponentStatus,
  RobotAssemblyState: RobotAssemblyState,
  IOStatus: IOStatus,
  NavigatorStates: NavigatorStates,
  DigitalIOStates: DigitalIOStates,
  IODeviceConfiguration: IODeviceConfiguration,
  CollisionAvoidanceState: CollisionAvoidanceState,
  CameraControl: CameraControl,
  EndpointState: EndpointState,
  IONodeStatus: IONodeStatus,
  CollisionDetectionState: CollisionDetectionState,
  EndpointNamesArray: EndpointNamesArray,
  HomingState: HomingState,
  IOComponentConfiguration: IOComponentConfiguration,
  IODeviceStatus: IODeviceStatus,
  DigitalIOState: DigitalIOState,
  JointCommand: JointCommand,
  URDFConfiguration: URDFConfiguration,
  EndpointStates: EndpointStates,
  NavigatorState: NavigatorState,
  IONodeConfiguration: IONodeConfiguration,
  IOComponentCommand: IOComponentCommand,
  AnalogIOStates: AnalogIOStates,
  JointLimits: JointLimits,
  CameraSettings: CameraSettings,
  HeadPanCommand: HeadPanCommand,
  InteractionControlState: InteractionControlState,
  InteractionControlCommand: InteractionControlCommand,
  SEAJointState: SEAJointState,
  IODataStatus: IODataStatus,
  DigitalOutputCommand: DigitalOutputCommand,
  HeadState: HeadState,
  AnalogOutputCommand: AnalogOutputCommand,
  HomingCommand: HomingCommand,
  CalibrationCommandActionFeedback: CalibrationCommandActionFeedback,
  CalibrationCommandActionGoal: CalibrationCommandActionGoal,
  CalibrationCommandResult: CalibrationCommandResult,
  CalibrationCommandActionResult: CalibrationCommandActionResult,
  CalibrationCommandGoal: CalibrationCommandGoal,
  CalibrationCommandAction: CalibrationCommandAction,
  CalibrationCommandFeedback: CalibrationCommandFeedback,
};
