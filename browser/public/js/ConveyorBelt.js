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
	this.speed_factor = 5;
	this.previous_speed = this.speed_current;
	this.direction = 0;
	this.unit_accel = 0;

	// Boxes on top of the conveyor belt.
	this.width_box = 12*module_obj.cw;
	this.length_box = 8*module_obj.ch;
	this.boxes = [];
	this.boxes_Ycenter = [];
	this.min_space_boxes = this.length_box*2;

	// Parameters for pickup area
	this.pickup_x = this.x_UL_belt - this.width_belt*0.15;
	this.pickup_y = this.y_UL_belt + this.length_belt*0.4;
	this.pickup_width = this.width_belt*1.3;
	this.pickup_length = this.length_belt*0.2;
	this.pickup_bottom_border = this.pickup_y+this.pickup_length;
	this.drawPickupArea = false;
	this.hasABox = false;
	this.ignoreThisBox = false;
	this.ignorePickupArea = false;

	// Temporary variables for testing stuff
	this.temp = this.direction;
	this.count_print = 0;

	self_conveyor = this;
}

ConveyorBelt.prototype.update = function (module_obj) {
	// Save current canvas context settings, which is used to restore later.
	module_obj.ctx.save();

	// Conveyor belt 
	module_obj.ctx.fillStyle = this.fillBelt;
	module_obj.ctx.fillRect(this.x_UL_belt, this.y_UL_belt, this.width_belt, this.length_belt);

	this.unit_accel = Math.abs(this.speed_current-this.speed_goal)>0 ? (this.speed_goal-this.speed_current)/(Math.abs(this.speed_current-this.speed_goal)*0.5) : 0;
	this.direction = (this.speed_current == 0) ? 0 : this.speed_current/Math.abs(this.speed_current);

	// Manipulate speed
	var speed_increment = Math.abs(this.speed_current-this.speed_goal)<0 ? 0 : 0.03*this.unit_accel;
	this.speed_current += speed_increment;
	if (Math.abs(this.speed_current)<0.01) { this.speed_current = 0; }
	
	// if (self_conveyor.count_print>=60) {
	// 	console.log(self_conveyor.boxes);
	// 	self_conveyor.count_print = 0;
	// } else { self_conveyor.count_print++; }

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

	self_conveyor.hasABox = false;

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

		self_conveyor.boxes_Ycenter[index] = this[index][1] + this[index][3]/2;
		
		var outOfBelt = this[index][1]+this[index][3] > (self_conveyor.y_UL_belt+self_conveyor.length_belt);
		if (!outOfBelt && place_this_box) {
			module_obj.ctx.fillStyle = self_conveyor.fillBox;
			module_obj.ctx.fillRect(item[0],item[1],item[2],item[3]);
		} else if (outOfBelt) {remove_a_box = true;}
		if (self_conveyor.drawPickupArea && !self_conveyor.ignorePickupArea && !self_conveyor.hasABox) {
			if (this[index][1]>self_conveyor.pickup_y && this[index][1]+self_conveyor.length_box<self_conveyor.pickup_bottom_border) {
				self_conveyor.hasABox = true;
				if (!self_conveyor.ignoreThisBox) {
					self_conveyor.stop();
				}
			}
		}
	}, this.boxes);
	// This has to be done outside of boxes.forEach loop to avoid the new 'first' box flickering.
	if (remove_a_box) {
		this.boxes.splice(0,1);
		this.boxes_Ycenter.splice(0,1);
	}

	// Draw the pickup region.
	if (this.drawPickupArea) {
		module_obj.ctx.beginPath();
		module_obj.ctx.setLineDash([1.0*module_obj.ch]);
		module_obj.ctx.lineWidth = 0.3*module_obj.ch;
		module_obj.ctx.strokeStyle = this.ignoreThisBox ? "#FF0000" : "#000000";
		module_obj.ctx.rect(this.pickup_x,this.pickup_y,this.pickup_width,this.pickup_length);
		module_obj.ctx.stroke();
	}

	// Restore canvas context settings. Any drawing commands should be above this line.
	module_obj.ctx.restore();
}

ConveyorBelt.prototype.stop = function() {
	this.previous_speed = this.speed_current==0 ? this.previous_speed : this.speed_current;
	this.speed_goal = 0;
}

ConveyorBelt.prototype.resume = function() {
	this.speed_goal = this.previous_speed;
}

ConveyorBelt.prototype.changeSpeed = function(amount) {
	this.speed_goal = Math.min(2, Math.max(-2, this.speed_goal+amount));
}

ConveyorBelt.prototype.setSpeed = function(value) {
	this.speed_goal = Math.min(2, Math.max(-2, value));
}

ConveyorBelt.prototype.addBox = function(random_x=false) {
	if (random_x) {
		var box_x_UL = this.x_UL_belt + (this.width_belt - this.width_box)*Math.random();
	} else {
		var box_x_UL = this.x_UL_belt + (this.width_belt - this.width_box)/2;
	}
	var box_y_UL = this.y_UL_belt;
	if (this.direction>=0) {
		this.boxes.push([box_x_UL, box_y_UL, this.width_box, this.length_box]);
		this.boxes_Ycenter.push(0);
	}
}

ConveyorBelt.prototype.pickupArea = function(ignorePickupArea=false) {
	this.drawPickupArea = !this.drawPickupArea;
	this.ignorePickupArea = this.drawPickupArea ? ignorePickupArea : false;
}

ConveyorBelt.prototype.ignorebox = function() {
	// This function currently switches between ignore and not ignore.
	// It is intended for testing purpose only and should not be used in this way for module 3.
	this.ignoreThisBox = !this.ignoreThisBox;
	console.log('ignore? ' + this.ignoreThisBox);
}

var conveyor = new ConveyorBelt(m);
