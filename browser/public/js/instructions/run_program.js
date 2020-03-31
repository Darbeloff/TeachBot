/**
 * Runs user program on robot.
 *
 * The last section of M2 uses this mode.
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.run_program = async function(instr, instructionAddr) {
	this.last_program = this.program.slice();
	var print_instructions = '';
	for (let i=0; i<self.program.length; i++) {
		if (self.program[i] === 'Toggle Gripper') {
			print_instructions += self.program[i] + ', ';
		} else {
			print_instructions += 'Go To Waypoint, '
		}
	}
	self.print(print_instructions);
	
	return new Promise(async (resolve, reject) => {
		for (let p=0; p<self.program.length; p++) {
			await new Promise(async (resolve2, reject2) => {
				this_step = self.program[p];
				if (this_step === 'Toggle Gripper') {
					await self.gripper(!self.gripper_closed);
					resolve2();
				} else {
					var goal = self.getGoToGoal(this_step, 0, false, 'audio');
					goal.on('result', result => { resolve2(); } );
					goal.send();
				}
			});
		}
		resolve();
	});
}