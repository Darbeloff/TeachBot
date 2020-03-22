/**
 * Allows user to program the robot.
 *
 * The last section of M2 uses this mode.
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.write_program = function(instr, instructionAddr) {
	this.free_mode = false
	this.program = [];

	// this.displayOff();
	// canvas_container.style.display = 'initial';
	// var multi_choice_url = DIR + 'images/new_button_box.JPG';
	// var program_url = DIR + 'images/program_rect.png';
	// display_choices(m.ctx, ['Toggle Gripper','Toggle Free Mode','Set Waypoint', 'Done', 'Remove Choice'], multi_choice_url);

	this.button_topic.subscribe(async function(message) {
		if (VERBOSE) console.log('Pressed: ' + message.data);

		switch (parseInt(message.data)) {
			case 5: 	// Done
				console.log(self.program)
				self.button_topic.unsubscribe();
				self.button_topic.removeAllListeners();
				self.displayOff(true);
				self.start(self.getNextAddress(instructionAddr));
				break;

			case 2: 	// Blue: Toggle Gripper
				self.program.push('Toggle Gripper')
				await self.gripper(!self.gripper_closed);
				break;

			case 3: 	// Black: Toggle Free Mode
				if (self.free_mode) {
					self.set_robot_mode({'mode': 'position'});
					self.free_mode = false;
				} else {
					self.set_robot_mode({
					'mode':'interaction ctrl', 
					'position_only':false, 
					'position_x': true,
					'position_y': true,
					'position_z': true,
					'orientation_x': true,
					'orientation_y': true,
					'orientation_z': true,
					'in_end_point_frame': false}, instructionAddr);
					self.free_mode = true;
				}
				break;

			case 4: 	// Yellow: Set Waypoint
				var current_pos = [];
				for (let j=0; j<JOINTS; j++) {
					current_pos[j] = self.dictionary[`JOINT_POSITION_${j}`];
				}
				self.program.push(current_pos);
				break;

			case -1: 	// Minus: Rm Command
				self.program.pop()
				break;

			default:
				console.log('No Support for this button');
				break;
		}

		var print_instructions = '';
		for (let i=0; i<self.program.length; i++) {
			if (self.program[i] === 'Toggle Gripper') {
				print_instructions += self.program[i] + ', ';
			} else {
				print_instructions += 'Go To Waypoint, '
			}
		}
		self.print(print_instructions);
	});
}