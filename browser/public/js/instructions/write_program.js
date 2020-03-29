/**
 * Allows user to program the robot.
 *
 * The last section of M2 uses this mode.
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.write_program = function(instr, instructionAddr) {
	console.log(`[${getRuntime()}] Writing new program:`);
	this.free_mode = false;
	var first_free_mode = true;
	var reset_ready = false;
	this.program = [];
	var ai_freemode = 5;
	var ai_minusreset = 6;

	// this.displayOff();
	// canvas_container.style.display = 'initial';
	// var multi_choice_url = DIR + 'images/new_button_box.JPG';
	// var program_url = DIR + 'images/program_rect.png';
	// display_choices(m.ctx, ['Toggle Gripper','Toggle Free Mode','Set Waypoint', 'Done', 'Remove Choice'], multi_choice_url);

	this.button_topic.subscribe(async function(message) {
		if (VERBOSE) console.log('Pressed: ' + message.data);
		player.src = DIR + 'audio/beep.mp3';

		if (parseInt(message.data)!==-1) reset_ready = false;

		switch (parseInt(message.data)) {
			case 5: 	// Red: Done
				console.log(`\t[${getRuntime()}] Playback:`);
				for (let p=0; p<self.program.length; p++) {
					console.log(`\t\t${self.program[p]}`)
				}

				self.set_robot_mode({'mode': 'position'});
				this.free_mode = false;
				self.button_topic.unsubscribe();
				self.button_topic.removeAllListeners();
				self.displayOff(true);
				self.start(self.getNextAddress(instructionAddr));

				break;

			case 2: 	// Blue: Toggle Gripper
				console.log(`\t[${getRuntime()}] Gripper`);

				player.play();
				self.program.push('Toggle Gripper')
				await self.gripper(!self.gripper_closed);

				break;

			case 3: 	// Black: Toggle Free Mode
				if (self.free_mode) {
					console.log(`\t[${getRuntime()}] Free Mode Off @ ${self.pose2str()}`);
					player.play();
					self.set_robot_mode({'mode': 'position'});
					self.free_mode = false;
				} else {
					console.log(`\t[${getRuntime()}] Free Mode On @ ${self.pose2str()}`);
					if (first_free_mode) {
						self.play(self.thisSection._audiofiles_mp3[ai_freemode], self.thisSection._audio_duration[ai_freemode], self.thisSection._textArray[ai_freemode]);
						first_free_mode = false;
					} else {
						player.play();
					}
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
				console.log(`\t[${getRuntime()}] Set Waypoint @ ${self.pose2str()}`);
				player.play();
				var current_pos = [];
				for (let j=0; j<JOINTS; j++) {
					current_pos[j] = self.dictionary[`JOINT_POSITION_${j}`];
				}
				self.program.push(current_pos);
				break;

			case -1: 	// Minus: Rm Command
				if (self.program.length > 0) {
					console.log(`\t[${getRuntime()}] Remove cmd #${self.program.length}: ${self.program.slice(self.program.length-1)}`);
					player.play();
					self.program.pop()
				} else {
					if (reset_ready) {
						console.log(`\t[${getRuntime()}] Reset to start position`);
						player.play();
						reset_ready = false;
						self.getGoToGoal('above_fourth_box_joint_arg').send();
					} else {
						reset_ready = true;
						self.play(self.thisSection._audiofiles_mp3[ai_minusreset], self.thisSection._audio_duration[ai_minusreset], self.thisSection._textArray[ai_minusreset]);
					}
				}
				
				break;

			case 1: 	// Plus: Recall Program
				let printStr = `\t[${getRuntime()}] Recall last program: [`;
				for (let p=0; p<self.program.length; p++) {
					printStr += `${self.program[p]}, `;
				}
				printStr = printStr.splice(printStr.length-2) + ']';
				console.log(printStr);
				player.play();
				self.program = self.last_program.slice();
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