/**
 * Changes the current graphics mode.
 *
 * Sets the graphic mode to one of the following:
 * - 'image': Displays an image.
 * - 'video': Plays a video.
 * - 'canvas': Displays the canvas and draws objects on it from the drawings array.
 * - 'multiple choice': Displays the multiple choice graphic with custom options.
 * - 
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.set_graphic_mode = function(instr, instructionAddr) {
	checkInstruction(instr, ['mode'], instructionAddr);
	if (VERBOSE) console.log(`Entering graphic mode: ${instr.mode}`);

	window.cancelAnimationFrame(this.canvas_frame_req);
	this.displayOff();
	this.graphic_mode = instr;

	switch (instr.mode) {
		case 'image':
			if (instr.hasOwnProperty('location')) {
				image.src = DIR + instr.location;
			}
			image.style.display = 'initial';

			break;

		case 'video':
			animator.style.display = 'initial';
			animator.play();
			
			break;

		case 'canvas':
			canvas_obj.style.display = 'initial';
			if (instr.hasOwnProperty('clear') && instr.clear) {
				this.drawings = [];
			}

			this.canvas_frame_req = window.requestAnimationFrame(function(timestamp) {
				self.drawCanvas(timestamp);
			});

			break;

		case 'multiple_choice':
			canvas_obj.style.display = 'initial';
			display_choices(m.ctx, ['Motors','Buttons','Cameras','Encoders','Wheels'], DIR + 'images/sized_cuff.png');

			break;

		case 'scroll wheel':
			canvas_obj.style.display = 'initial';
			run_odometer = true;
			canvas_obj.width = canvas_obj.width;
			init_odometer(this.ctx);

			break;

		default:
			throw `Graphic mode ${instr.mode} is not supported.`;
	}
};

Module.prototype.drawCanvas = function(timestamp) {
	this.ctx.clearRect(0,0,100*this.cw,100*this.ch);

	this.drawings.forEach(function(obj, ind) {
		switch (obj.shape) {
			case 'ball':
				var cx = eval(self.hashTokeyVal(obj.cx))*self.cw;
				var cy = eval(self.hashTokeyVal(obj.cy))*self.ch;
				var r = eval(self.hashTokeyVal(obj.r))*self.ch;
				var fillStyle = obj.fillStyle;

				draw_ball(self.ctx, cx, cy, r, fillStyle, obj.label);

				break;

			case 'bar':
				var x = eval(self.hashTokeyVal(obj.x))*self.cw;
				var y = eval(self.hashTokeyVal(obj.y))*self.ch;
				var width = eval(self.hashTokeyVal(obj.width))*self.cw;
				var max_height = eval(self.hashTokeyVal(obj.max_height))*self.ch;
				var height_percent = eval(self.hashTokeyVal(obj.height_percent));
				var fillStyle = eval(self.hashTokeyVal(obj.fillStyle));

				draw_bar_new(self.ctx, x, y, width, max_height, height_percent, fillStyle, obj.label, obj.label_align);

				break;

			case 'arc':
				var x1 = eval(self.hashTokeyVal(obj.x1))*self.cw;
				var y1 = eval(self.hashTokeyVal(obj.y1))*self.ch;
				var x2 = eval(self.hashTokeyVal(obj.x2))*self.cw;
				var y2 = eval(self.hashTokeyVal(obj.y2))*self.ch;
				var x3 = eval(self.hashTokeyVal(obj.x3))*self.cw;
				var y3 = eval(self.hashTokeyVal(obj.y3))*self.ch;
				var ccw = eval(self.hashTokeyVal(obj.ccw));

				arc3pt(self.ctx, x1, y1, x2, y2, x3, y3, ccw);

				break;
		}
	});

	this.canvas_frame_req = window.requestAnimationFrame(function(timestamp) {
		self.drawCanvas(timestamp);
	});
}