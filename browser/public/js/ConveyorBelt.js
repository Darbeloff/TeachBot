function ConveyorBelt(module_obj) {
	// Initialize key features of the conveyor belt.
	// UL = Upper-Left, it is the starting point of the drawing.
	this.x_UL_belt = 75*module_obj.cw;
	this.y_UL_belt = 5*module_obj.ch;
	this.width_belt = 18.0*module_obj.cw;
	this.length_belt = 90.0*module_obj.ch;
	this.strap_size = 2*module_obj.ch;
	this.fillBelt = "#222222";
	this.fillStrap = "#AAAAAA";
	this.fillBox = "#5566CC"
	
	// Create straps on the conveyor belt.
	this.strap_interval = 10*module_obj.ch;
	this.straps_y = [];
	for (var i=this.y_UL_belt;i<this.length_belt;i+=this.strap_interval) {
		this.straps_y.push(i);
	}
	this.strap_begin = this.y_UL_belt;
	this.strap_end = this.strap_begin + this.length_belt - this.strap_size;

	// Status variables
	this.speed_current = 0;
	this.speed_goal = 0;
	this.speed_goal_prev = 0;
	this.direction = 0;
	this.unit_accel = 0;

	// Boxes on top of the conveyor belt.
	this.width_box = 12*module_obj.cw;
	this.length_box = 8*module_obj.ch;
	this.boxes = [];
	this.min_space_boxes = this.length_box*1.5;

	// Parameters for pickup area
	this.pickup_x = this.x_UL_belt - this.width_belt*0.15;
	this.pickup_y = this.y_UL_belt + this.length_belt*0.44;
	this.pickup_width = this.width_belt*1.3;
	this.pickup_length = this.length_belt*0.2;
	this.pickup_bottom_border = this.pickup_y+this.pickup_length;
	this.drawPickupArea = false;
	this.suppressPickup = false;
	this.box_in_pickup = false;

	// Inspected boxes, mostly manipulated in GripperProjection.js
	this.drawInspectedArea = false;
	this.x_inspected = 5*module_obj.cw;
	this.y_inspected = 10*module_obj.ch;
	this.width_inspected = 30*module_obj.cw;
	this.height_inspected = 70*module_obj.ch;
	this.inspected_boxes = [];
	this.inside_inspected = false;

	// Timing feature
	this.timing_mode = false;
	this.start_time = 0;
	this.is_moving = false;
	this.move_speed = 0;
	this.move_duration = 0;
	this.stop_duration = 0;
	this.repeat = false;
	this.started = false;
	this.timing_add_box = false;

	// Light
	this.light_on = new Image();
	this.light_on.src = DIR + 'images/light_on.png';
	this.light_off = new Image();
	this.light_off.src = DIR + 'images/light_off.png';
	this.status_light = 0; // 0:disabled; -1:off; 1:on

	// Ready/Busy states
	this.rb_conveyor = false;
	this.ready_conveyor = false;

	// Temporary variables for testing stuff
	this.count_print = 0;
	this.inc = 1;
	this.print;

	// Module-specific command variables which have unique behaviors. Will be generalized if necessary in the future.
	this.command_variables = {
		// Currently not in use, and should not be used.
		m3_mess_box_placement: {
			do: false,
			random: this.move_duration*Math.random(),
			box_placed: false
		},
		m3_mess_belt_speed: {
			do:false
		}
	}

	self_conveyor = this;
}

// may not work due to the property type of object command_variables (var or string)
ConveyorBelt.prototype.setCommandVariable = function(command, action) {
	if (this.command_variables.hasOwnProperty(command)) {
		this.command_variables[command].do = action;
	} else {
		throw `Command ${command} not found!`;
	}
}

ConveyorBelt.prototype.timing = function(timing_mode, move_speed=0, move_duration=0, stop_duration=0, add_box=false, repeat=false) {
	this.timing_mode = timing_mode;
	if (this.timing_mode) {
		this.is_moving = true;
		this.started = true;
		if (this.started) {
			this.start_time = new Date().getTime();
		}
		if (move_duration>0) {
			console.log('Timing feature turned on.');
			if (this.command_variables.m3_mess_belt_speed.do==true) {
				this.move_speed = 0.6+1.4*Math.random();
				this.move_duration = 10/(2*this.move_speed);
			} else {
				this.move_speed = move_speed;
				this.move_duration = move_duration;
			}
			this.stop_duration = stop_duration;
			this.repeat = repeat;
			this.timing_add_box = add_box;
			if (typeof this.command_variables.m3_mess_box_placement !== 'undefined') {
				// this.command_variables.m3_mess_box_placement.random = this.move_duration*Math.random();
				this.command_variables.m3_mess_box_placement.random = 3.0;
			}
			if (this.timing_add_box && this.command_variables.m3_mess_box_placement.do==false) { self_conveyor.addBox(); }
		} else {
			this.timing_mode = false;
			this.is_moving = false;
			this.initialized = false;
			throw 'Moving duration cannot be zero or negative!';
		}
	} else {
		console.log('Timing feature turned off.');
		this.is_moving = false;
		this.start_time = 0;
		this.move_speed = 0;
		this.move_duration = 0;
		this.stop_duration = 0;
		this.repeat = false;
		this.started = false;
		this.timing_add_box = false;
	}
}

ConveyorBelt.prototype.checkTiming = function() {
	if (this.timing_mode) {
		if (this.started) {
			if (new Date().getTime()-this.start_time>(this.is_moving?this.move_duration*1000:this.stop_duration*1000)){
				this.is_moving = !this.is_moving;
				this.start_time = new Date().getTime()
				if (this.is_moving && this.timing_add_box && this.command_variables.m3_mess_box_placement.do==false) { 
					self_conveyor.addBox();
				}
				if (this.is_moving && this.command_variables.m3_mess_box_placement.do==true) {
					this.command_variables.m3_mess_box_placement.box_placed = false;
					this.command_variables.m3_mess_box_placement.random = this.move_duration*Math.random();
					console.log(this.command_variables.m3_mess_box_placement.random)
				}
				if (!this.is_moving && this.command_variables.m3_mess_belt_speed.do==true) {
					this.move_speed = 0.6+1.4*Math.random();
					console.log(this.move_speed)
					this.move_duration = 10/(2*this.move_speed);
				}
				if (this.status_light!==0) {
					if (this.is_moving) {this.status_light = -1;}
					else { this.status_light = 1;}
				}
			} else {
				this.speed_goal = this.is_moving?this.move_speed:0;
				if (this.is_moving && this.command_variables.m3_mess_box_placement.do==true) {
					if (new Date().getTime()-this.start_time>this.command_variables.m3_mess_box_placement.random*1000 && this.command_variables.m3_mess_box_placement.box_placed==false) {
						self_conveyor.addBox();
						this.command_variables.m3_mess_box_placement.box_placed = true;
					}
				}
			}
		} else {
			this.started = this.speed_current!==0;
			if (this.print) {console.log('Timing on, move the conveyor belt to begin.');}
		}
	} else {
		if (this.print) {console.log('Timing feature off.');}
	}
}

ConveyorBelt.prototype.update = function (module_obj) {
	this.print = this.count_print==60;
	this.count_print = this.count_print<60 ? this.count_print+this.inc : 0;
	
	this.checkTiming();

	// Save current canvas context settings, which is used to restore later.
	module_obj.ctx.save();

	// Conveyor belt 
	module_obj.ctx.fillStyle = this.fillBelt;
	module_obj.ctx.fillRect(this.x_UL_belt, this.y_UL_belt, this.width_belt, this.length_belt);

	// Manipulate speed
	if (this.drawPickupArea && !this.suppressPickup && !this.box_in_pickup) {
		if (this.speed_goal==0 && this.speed_goal_prev!==0) {
			var temp = this.speed_goal;
			this.speed_goal = this.speed_goal_prev;
			this.speed_goal_prev = temp;
			if (this.rb_conveyor) { this.ready_conveyor = false; }
			if (this.status_light!==0) {this.status_light = -1;}
		}
	}

	var speed_goal_converted = this.length_belt/10/60*this.speed_goal;
	this.unit_accel = Math.abs(this.speed_current-speed_goal_converted)>0 ? (speed_goal_converted-this.speed_current)/(Math.abs(this.speed_current-speed_goal_converted)*0.5) : 0;
	this.direction = (this.speed_current==0) ? 0 : this.speed_current/Math.abs(this.speed_current);
	var speed_increment = Math.abs(this.speed_current-speed_goal_converted)<0 ? 0 : 0.03*this.unit_accel;
	this.speed_current += speed_increment;
	if (Math.abs(this.speed_current)<0.01) { this.speed_current = 0; }
	
	// Straps on the belt
	module_obj.ctx.fillStyle = this.fillStrap;
	self_conveyor.straps_y.forEach(function (item, index) {
		if (self_conveyor.direction==1) {
			if (item>=self_conveyor.strap_begin && item<=self_conveyor.strap_end) {
				this[index] = item+self_conveyor.speed_current;
				if (this[index]<=self_conveyor.strap_end) {
					module_obj.ctx.fillRect(self_conveyor.x_UL_belt, this[index], self_conveyor.width_belt, self_conveyor.strap_size);
				}
			} else {
				if (item<self_conveyor.strap_begin) { this[index] = self_conveyor.strap_end*1.01; }
				if (Math.min(...this)>=self_conveyor.strap_begin+self_conveyor.strap_interval && Math.min(...this)>self_conveyor.strap_begin) {
					this[index] = self_conveyor.strap_begin;
					module_obj.ctx.fillRect(self_conveyor.x_UL_belt, this[index], self_conveyor.width_belt, self_conveyor.strap_size);
				}
			}
		} else if (self_conveyor.direction==-1){
			if (item>=self_conveyor.strap_begin && item<=self_conveyor.strap_end) {
				this[index] = item+self_conveyor.speed_current;
				if (this[index]>=self_conveyor.strap_begin) {
					module_obj.ctx.fillRect(self_conveyor.x_UL_belt, this[index], self_conveyor.width_belt, self_conveyor.strap_size);
				}
			} else {
				if (item>self_conveyor.strap_end) { this[index] = self_conveyor.strap_begin*0.99; }
				if (Math.max(...this)<=self_conveyor.strap_end-self_conveyor.strap_interval && Math.max(...this)<self_conveyor.strap_end) {
					this[index] = self_conveyor.strap_end;
					module_obj.ctx.fillRect(self_conveyor.x_UL_belt, this[index], self_conveyor.width_belt, self_conveyor.strap_size);
				}
			}
		} else {
			if (item>=self_conveyor.strap_begin && item<=self_conveyor.strap_end) {
				module_obj.ctx.fillRect(self_conveyor.x_UL_belt, this[index], self_conveyor.width_belt, self_conveyor.strap_size);
			}
		}
	}, self_conveyor.straps_y);

	var remove_a_box = false;
	var place_this_box = false;
	this.box_in_pickup = false;

	this.boxes.forEach(function (item, index) {
		if (index>0){
			if(this[index-1][1]-self_conveyor.y_UL_belt>self_conveyor.min_space_boxes) {
				this[index][1] += self_conveyor.speed_current;
				place_this_box = true;
			} else {place_this_box = false;}
		} else {
			this[index][1] += self_conveyor.speed_current;
			place_this_box = true;
		}
		
		var outOfBelt = this[index][1]+this[index][3] > (self_conveyor.y_UL_belt+self_conveyor.length_belt);
		if (!outOfBelt && place_this_box) {
			module_obj.ctx.fillStyle = self_conveyor.fillBox;
			module_obj.ctx.fillRect(item[0],item[1],item[2],item[3]);
		} else if (outOfBelt) {remove_a_box = true;}

		if (self_conveyor.drawPickupArea && !self_conveyor.suppressPickup) {
			if (this[index][1]>self_conveyor.pickup_y && this[index][1]+self_conveyor.length_box<self_conveyor.pickup_bottom_border) {
				self_conveyor.box_in_pickup = true;
				if (self_conveyor.speed_goal!==0) {
					var temp = self_conveyor.speed_goal;
					self_conveyor.speed_goal = 0;
					self_conveyor.speed_goal_prev = temp;
					if (self_conveyor.rb_conveyor) {self_conveyor.ready_conveyor = true;}
					if (self_conveyor.status_light!==0) {self_conveyor.status_light = 1; }
				}
			}
		}
	}, this.boxes);
	// This has to be done outside of boxes.forEach loop to avoid the new 'first' box flickering.
	if (remove_a_box) {
		this.boxes.splice(0,1);
	}

	// Draw boxes in the inspected area
	this.inspected_boxes.forEach(function (item, index) {
		module_obj.ctx.fillStyle = self_conveyor.fillBox;
		module_obj.ctx.fillRect(item[0],item[1],-item[2],item[3]);
	}, this.inspected_boxes)

	// Draw the pickup region.
	if (this.drawPickupArea) {
		module_obj.ctx.beginPath();
		module_obj.ctx.setLineDash([1.0*module_obj.ch]);
		module_obj.ctx.lineWidth = 0.3*module_obj.ch;
		module_obj.ctx.strokeStyle = "#000000";
		module_obj.ctx.rect(this.pickup_x,this.pickup_y,this.pickup_width,this.pickup_length);
		module_obj.ctx.stroke();
	}

	// Draw the area for storing inspected goods
	if (this.drawInspectedArea) {
		module_obj.ctx.beginPath();
		module_obj.ctx.setLineDash([1.0*module_obj.ch]);
		module_obj.ctx.lineWidth = 0.3*module_obj.ch;
		module_obj.ctx.strokeStyle = "#000000";
		module_obj.ctx.rect(5*module_obj.cw, 10*module_obj.ch, 30*module_obj.cw, 70*module_obj.ch);
		module_obj.ctx.stroke();
	}

	if (Boolean(this.status_light)) {
		var on_off = this.status_light>0?this.light_on:this.light_off;
		module_obj.ctx.drawImage(on_off,65*module_obj.cw,72*module_obj.ch, 18*module_obj.ch, 18*module_obj.ch);
	}

	if (this.rb_conveyor) {
		module_obj.ctx.fillStyle = 'black';
		var font_num = Math.round(2*module_obj.cw);
		module_obj.ctx.font = font_num.toString() + "px Arial";
		if (this.ready_conveyor) {
			module_obj.ctx.fillText('ready',66.5*module_obj.cw,93.5*module_obj.ch);
		} else {
			module_obj.ctx.fillText('busy',66.9*module_obj.cw,93.5*module_obj.ch);
		}
	}

	// Restore canvas context settings. Any drawing commands should be above this line.
	module_obj.ctx.restore();
}

ConveyorBelt.prototype.stop = function(clearBox=false) {
	this.speed_goal_prev = this.speed_goal;
	this.speed_goal = 0;
	if (clearBox) {
		this.boxes = [];
	}
	this.timing(false);
}

ConveyorBelt.prototype.resume = function() {
	this.speed_goal = this.speed_goal_prev;
}

ConveyorBelt.prototype.changeSpeed = function(amount) {
	this.speed_goal = Math.min(5, Math.max(-5, this.speed_goal+amount));
}

ConveyorBelt.prototype.setSpeed = function(value) {
	this.speed_goal = Math.min(5, Math.max(-5, value));
}

ConveyorBelt.prototype.addBox = function(random_x=false) {
	if (random_x) {
		var box_x_UL = this.x_UL_belt + (this.width_belt - this.width_box)* Math.min(Math.max(0.2,Math.random()), 0.8);
	} else {
		var box_x_UL = this.x_UL_belt + (this.width_belt - this.width_box)/2;
	}
	var box_y_UL = this.y_UL_belt;
	if (this.direction>=0) {
		this.boxes.push([box_x_UL, box_y_UL, this.width_box, this.length_box]);
	}
}

ConveyorBelt.prototype.removeBox = function() {
	this.boxes.splice(0,1);
}

ConveyorBelt.prototype.pickupArea = function(suppress=false) {
	this.drawPickupArea = !this.drawPickupArea;
	this.suppressPickup = this.drawPickupArea?suppress:false;
}

ConveyorBelt.prototype.inspectedArea = function() {
	this.drawInspectedArea = !this.drawInspectedArea;
}

ConveyorBelt.prototype.light = function() {
	this.status_light = this.status_light==0?-1:0;
}

ConveyorBelt.prototype.readyBusy = function() {
	this.rb_conveyor = !this.rb_conveyor;
	GP.display_rb = !GP.display_rb;
}

// This function changes the state of robot. It is implemented in the conveyor belt file
// because GripperProjection does not have an instruction case that executes a command.
ConveyorBelt.prototype.robotReady = function(state) {
	GP.robot_ready = state;
}

var conveyor = new ConveyorBelt(m);
