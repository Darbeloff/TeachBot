/**
 * Changes the current graphics mode.
 *
 * Sets the graphic mode to one of the following:
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
			checkInstruction(instr, ['joints', 'resetPOS'], instructionAddr);
			['joints', 'resetPOS', 'bias', 'F2V', 'min_thresh'].forEach(function(attr, i) {
				if (instr.hasOwnProperty(attr)) req[attr] = instr[attr];
			});
			break;

		case 'impedance ctrl':
			checkInstruction(instr, ['joints', 'resetPOS', 'V2F', 'X2F'], instructionAddr);
			['joints', 'resetPOS', 'V2F', 'X2F'].forEach(function(attr, i) {
				req[attr] = instr[attr];
			});
			break;

		case 'interaction ctrl':
			break;

		default:
			throw `Robot mode not supported.`
	}

	this.robot_mode = req;

	this.SetRobotModeSrv.callService(new ROSLIB.ServiceRequest(req), result => {
		if (VERBOSE) console.log('Success')
	});
};