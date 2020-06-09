/**
 * Gets numeric user input.
 *
 * Sets graphic mode to 'numeric input' and receives input from the Arduino box.
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.numeric_input = function(instr, instructionAddr) {
	var currentGraphicMode = this.graphic_mode;
	this.set_graphic_mode({'mode': 'numeric input'}, instructionAddr);

	this.button_topic.subscribe(async function(message) {
		value = parseInt(message.data)
		if (value > 1){
			if (VERBOSE) console.log('Finished');
			if (instr.hasOwnProperty('store_answer_in')) {
				self.dictionary[instr.store_answer_in] = wheel_val;
			}
			self.button_topic.unsubscribe();
			self.button_topic.removeAllListeners();
			self.set_graphic_mode(currentGraphicMode, instructionAddr);
			self.start(self.getNextAddress(instructionAddr));
		}else{
			wheel_val+= value;
			draw_odometer(m.ctx, odometer_url, wheel_val);
			if (VERBOSE) console.log('Wheel value: ' + wheel_val);
		}
	});
}