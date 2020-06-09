/**
 * Draw on the canvas.
 *
 * Registers drawable object to drawings array.
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.draw = function(instr, instructionAddr) {
	checkInstruction(instr, ['shape'], instructionAddr);

	switch (instr.shape) {
		case 'ball':
			checkInstruction(instr, ['cx', 'cy', 'r', 'fillStyle'], instructionAddr);
			break;

		case 'bar':
			checkInstruction(instr, ['x', 'y', 'width', 'max_height', 'height_percent', 'fillStyle'], instructionAddr);
			break;

		case 'arc':
			checkInstruction(instr, ['x1', 'y1', 'x2', 'y2', 'x3', 'y3', 'ccw'], instructionAddr);
			break;

		case 'rectangle':
			checkInstruction(instr, ['x', 'y', 'width', 'height'], instructionAddr);
			break;

		default:
			throw `Draw ${instr.shape} not yet supported.`
	}

	this.drawings.push(instr);

	this.start(this.getNextAddress(instructionAddr));
}