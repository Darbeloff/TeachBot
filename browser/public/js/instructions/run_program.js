/**
 * Runs user program on robot.
 *
 * The last section of M2 uses this mode.
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.run_program = async function(instr, instructionAddr) {
	return new Promise(async (resolve, reject) => {
		for (let p=0; p<self.program.length; p++) {
			var step = new Promise(async (resolve2, reject2) => {
				this_step = self.program[p];
				if (this_step === 'Toggle Gripper') {
					await self.gripper(!self.gripper_closed);
					resolve2();
				} else {
					var goal = getGoToGoal(this_step);
					goal.on('result', result => { resolve2(); } );
					goal.send();
				}
			});

			await step;
		}
		resolve();
	});
}