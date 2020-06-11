/**
 * Changes the current graphics mode.
 *
 * Sets the graphic mode to one of the following:
 * - 'image': Displays an image.
 * - 'video': Plays a video.
 * - 'canvas': Displays the canvas and draws objects on it from the drawings array.
 * - 'multiple choice': Displays the multiple choice graphic with custom options.
 * - 'numeric input': Displays the numeric input graphic with counter.
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
			var image = document.getElementById(HTML_IMG);
			if (instr.hasOwnProperty('src')) {
				image.replaceWith(this.media.get(DIR+instr.src));
				image = document.getElementById(HTML_IMG);
			}
			image.style.display = 'initial';

			break;

		case 'video':
			var video = document.getElementById(HTML_VIDEO);
			video.replaceWith(this.media.get(DIR+instr.src));
			video = document.getElementById(HTML_VIDEO);
			video.style.display = 'initial';
			video.play();
			
			break;

		case 'camerabw_graphic':
				document.getElementById('canvas_container').style.display = 'initial';
				var cv_image_url = DIR + 'images/cv_1.png';
				var bw_image = new Image();

				bw_image.onload = function() {
					console.log('Displaying camera')
		            self.ctx.drawImage(bw_image, 300, 10, self.ctx.canvas.width*.7, self.ctx.canvas.height)
				};

				bw_image.src = cv_image_url;

				break;

		case 'camera_color_graphic':
				document.getElementById('canvas_container').style.display = 'initial';
				var cv_image_url = DIR + 'images/cv_2.png';
				var color_image = new Image();

				color_image.onload = function() {
					console.log('Displaying camera')
		            self.ctx.drawImage(color_image, 300, 10, self.ctx.canvas.width*.7, self.ctx.canvas.height)
				};

				color_image.src = cv_image_url;

				break;

		case 'canvas':
			document.getElementById('canvas_container').style.display = 'initial';
			if (instr.hasOwnProperty('clear') && instr.clear) {
				this.drawings = [];
			}

			if (instr.hasOwnProperty('custom')) {
				if (!instr.custom) {
					this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
					this.canvas_frame_req = window.requestAnimationFrame(function(timestamp) {
						self.drawCanvas(timestamp);
					});
				}
			} else {
				this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
				this.canvas_frame_req = window.requestAnimationFrame(function(timestamp) {
					self.drawCanvas(timestamp);
				});
			}

			break;

		case 'multiple choice':
			document.getElementById('canvas_container').style.display = 'initial';
			display_choices(m.ctx, ['Motors','Buttons','Cameras','Encoders','Wheels'], DIR + 'images/new_button_box.JPG');

			break;

		case 'numeric input':
			document.getElementById('canvas_container').style.display = 'initial';
			wheel_val = 0
			draw_odometer(m.ctx, odometer_url, wheel_val);
			//document.getElementById(HTML_CANVAS).width = document.getElementById(HTML_CANVAS).width;

			break;

		case 'pos_orient':
			document.getElementById('canvas_container').style.display = 'initial';
			var orient_bw_url = DIR + 'images/orientation_bw.png';
			var orient_color_url = DIR + 'images/orientation_color.png';
			var position_bw_url = DIR + 'images/position_bw.png';
			var position_color_url = DIR + 'images/position_color.png';

			draw_pos_orien(m.ctx,3,300,400,position_color_url,position_bw_url, orient_color_url, orient_bw_url);

			break;

		case 'projection':
			document.getElementById('canvas_container').style.display = 'initial';

			this.position.subscribe(async function(message) {
					// if (VERBOSE) console.log(message.j1);
					draw_goal(self.ctx, 100, message.j1*400+100)
				});

			this.button_topic.subscribe(async function(message) {
					console.log('here')
					self.position.unsubscribe();
					self.position.removeAllListeners();
					self.button_topic.unsubscribe();
					self.button_topic.removeAllListeners();
				});

			break;

		case 'show_camera':

				var cv_image_url = DIR + 'images/cv_image.png';
				var cv_image = new Image();

				cv_image.onload = function() {
					console.log('Displaying camera')
		            self.ctx.drawImage(cv_image, 300, 10, self.ctx.canvas.width*.75, self.ctx.canvas.height*.95)
				};

				cv_image.src = cv_image_url;

				break;

		default:
			throw `Graphic mode ${instr.mode} is not supported.`;
	}
	
};

Module.prototype.drawCanvas = function(timestamp) {
	document.getElementById(HTML_CANVAS).style.display = 'initial';
	this.ctx.clearRect(0,0,100*this.cw,100*this.ch);

	this.drawings.forEach(function(obj, ind) {
		switch (obj.shape) {
			case 'ball':
				var cx = eval(self.hash2keyVal(obj.cx))*self.cw;
				var cy = eval(self.hash2keyVal(obj.cy))*self.ch;
				var r = eval(self.hash2keyVal(obj.r))*self.ch;
				var fillStyle = obj.fillStyle;

				draw_ball(self.ctx, cx, cy, r, fillStyle, obj.label);

				break;

			case 'bar':
				var x = eval(self.hash2keyVal(obj.x))*self.cw;
				var y = eval(self.hash2keyVal(obj.y))*self.ch;
				var width = eval(self.hash2keyVal(obj.width))*self.cw;
				var max_height = eval(self.hash2keyVal(obj.max_height))*self.ch;
				var height_percent = eval(self.hash2keyVal(obj.height_percent));
				var fillStyle = eval(self.hash2keyVal(obj.fillStyle));

				draw_bar_new(m.ctx, x, y, width, max_height, height_percent, fillStyle, obj.label, obj.label_align);

				break;

			case 'arc':
				var x1 = eval(self.hash2keyVal(obj.x1))*self.cw;
				var y1 = eval(self.hash2keyVal(obj.y1))*self.ch;
				var x2 = eval(self.hash2keyVal(obj.x2))*self.cw;
				var y2 = eval(self.hash2keyVal(obj.y2))*self.ch;
				var x3 = eval(self.hash2keyVal(obj.x3))*self.cw;
				var y3 = eval(self.hash2keyVal(obj.y3))*self.ch;
				var ccw = eval(self.hash2keyVal(obj.ccw));

				arc3pt(self.ctx, x1, y1, x2, y2, x3, y3, ccw);

				break;
				
			case 'rectangle':
				var x = eval(self.hash2keyVal(obj.x))*self.cw;
				var y = eval(self.hash2keyVal(obj.y))*self.ch;
				var width = eval(self.hash2keyVal(obj.width))*self.cw;
				var height = eval(self.hash2keyVal(obj.height))*self.ch;
				if (obj.hasOwnProperty('rotate')) {
					var rotate = eval(self.hash2keyVal(obj.rotate));
					draw_rectangle(self.ctx, x, y, width, height, rotate);
				} else {
					draw_rectangle(self.ctx, x, y, width, height);
				}

				break;
		}
	});

	this.canvas_frame_req = window.requestAnimationFrame(function(timestamp) {
		self.drawCanvas(timestamp);
	});
}