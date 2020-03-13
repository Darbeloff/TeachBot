/**
 * Changes the current robot mode.
 *
 * Sets the robot mode to one of the following:
 * - 'image': Displays an image.
 * - 'video': Plays a video.
 * - 'canvas': Displays the canvas and draws objects on it from the drawings array.
 * - 'multiple choice': Displays the multiple choice graphic with custom options.
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.set_robot_mode = function(instr, instructionAddr) {
	checkInstruction(instr, ['mode'], instructionAddr);
	if (VERBOSE) console.log(`Entering robot mode: ${instr.mode}`);

	var req = {mode: instr.mode};
	switch (instr.mode) {
		case 'position':
			break;

		case 'admittance ctrl':
			checkInstruction(instr, ['joints', 'resetPos'], instructionAddr);
			['joints', 'resetPos', 'bias', 'F2V', 'min_thresh'].forEach(function(attr, i) {
				if (instr.hasOwnProperty(attr)) req[attr] = instr[attr];
			});
			break;

		case 'impedance ctrl':
			checkInstruction(instr, ['joints', 'resetPos', 'V2F', 'X2F'], instructionAddr);
			['joints', 'resetPos', 'V2F', 'X2F'].forEach(function(attr, i) {
				req[attr] = instr[attr];
			});
			break;

		case 'interaction ctrl':
			['position_only', 'orientation_only', 'plane_horizontal', 'plane_vertical_xz',
			 'plane_vertical_yz', 'nullspace_only', 'position_x', 'position_y',
			 'position_z', 'orientation_x', 'orientation_y', 'orientation_z',
			 'constrained_axes', 'in_endpoint_frame', 'interaction_frame', 'K_nullspace',
			 'rate'].forEach(function(attr, i) {
			 	if (instr.hasOwnProperty(attr)) req[attr] = instr[attr];
			});
			break;

		default:
			throw `Robot mode "${instr.mode}" not supported.`
	}

	this.robot_mode = req;

	return new Promise ((resolve, reject)=>{this.SetRobotModeSrv.callService(new ROSLIB.ServiceRequest(req), result => { resolve(); })});
};