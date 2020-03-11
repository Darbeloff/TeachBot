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
			var step = new Promise((resolve2, reject2) => {
				this_step = self.program[p];
				if (this_step === 'Open Gripper') {
					var goal = new ROSLIB.Goal({
						actionClient: this.GripperAct,
						goalMessage:{grip: false}
					});
					goal.on('result', result => { resolve2(); });
					goal.send();
				} else if (this_step === 'Close Gripper') {
					var goal = new ROSLIB.Goal({
						actionClient: this.GripperAct,
						goalMessage:{grip: true}
					});
					goal.on('result', result => { resolve2(); });
					goal.send();
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