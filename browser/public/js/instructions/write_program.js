/**
 * Allows user to program the robot.
 *
 * The last section of M2 uses this mode.
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.write_program = function(instr, instructionAddr) {
	this.displayOff();
	canvas_container.style.display = 'initial';
	var multi_choice_url = DIR + 'images/new_button_box.JPG';
	var program_url = DIR + 'images/program_rect.png';
	this.free_mode = false
	this.program = [];

	display_choices(m.ctx, ['Open Gripper','Close Gripper','Free Mode', 'Done', 'Remove Choice'], multi_choice_url);

	this.button_topic.subscribe(async function(message) {
		if (VERBOSE) console.log('Pressed: ' + message.data);

		if (self.free_mode) {
			console.log('entering position mode')
			display_choices(m.ctx, ['Open Gripper','Close Gripper','Free Mode', 'Done', 'Remove Choice'], multi_choice_url, code=true, self.program, 10, 80);
			self.program.push([self.dictionary['JOINT_POSITION_0'],self.dictionary['JOINT_POSITION_1'],self.dictionary['JOINT_POSITION_2'],self.dictionary['JOINT_POSITION_3'],self.dictionary['JOINT_POSITION_4'],self.dictionary['JOINT_POSITION_5'],self.dictionary['JOINT_POSITION_6']])
			self.set_robot_mode({
				'mode':'position', 
				'ways':true}, instructionAddr);
			self.free_mode = false
		} else {
			switch (parseInt(message.data)) {
				case 5: 	// Done
					console.log(self.program)
					self.button_topic.unsubscribe();
					self.button_topic.removeAllListeners();
					self.displayOff(true);
					self.start(self.getNextAddress(instructionAddr));
					break;

				case 2: 	// Open Gripper
					self.program.push('Open Gripper')
					var goal_Gripper = new ROSLIB.Goal({
						actionClient: self.GripperAct,
						goalMessage:{grip: false}
					});
					goal_Gripper.on('result', function(result){
						display_choices(m.ctx, ['Open Gripper','Close Gripper','Free Mode', 'Done', 'Remove Choice'], multi_choice_url, code=true, self.program, 10, 80);
					});
					goal_Gripper.send();
					break;

				case 3: 	// Close Gripper
					self.program.push('Close Gripper')
					var goal_Gripper = new ROSLIB.Goal({
						actionClient: self.GripperAct,
						goalMessage:{grip: true}
					});
					goal_Gripper.on('result', function(result){
						display_choices(m.ctx, ['Open Gripper','Close Gripper','Free Mode', 'Done', 'Remove Choice'], multi_choice_url, code=true, self.program, 10, 80);
					});
					goal_Gripper.send();
					break;

				case 4: 	// Set Waypoint
					console.log('entering free mode')
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
				
					self.ctx.clearRect(0,0,100*self.cw,100*self.ch);
					canvas_container.style.display = 'initial';
					self.free_mode = true;
					break;

				case -1: 	// Rm Command
					self.program.pop()
					display_choices(m.ctx, ['Open Gripper','Close Gripper','Free Mode', 'Done', 'Remove Choice'], multi_choice_url, code=true, self.program, 10, 80);
					break;

				default:
					console.log('No Support for this button');
					break;
		}
	}
}