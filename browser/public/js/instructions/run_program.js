/**
 * Runs user program on robot.
 *
 * The last section of M2 uses this mode.
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.run_program = async function(instr, instructionAddr) {
	for (let p=0; p<self.program.length; p++) {
		var step = new Promise((resolve, reject) => {
			this_step = self.program[p];
			if (this_step === 'Open Gripper') {
				var goal = new ROSLIB.Goal({
					actionClient: this.GripperAct,
					goalMessage:{grip: false}
				});
				goal_Gripper2.on('result', function(result){
					resolve();
				});
				goal_Gripper2.send();
			} else if (this_step === 'Close Gripper') {
				
			} else {

			}
		});
	}
}