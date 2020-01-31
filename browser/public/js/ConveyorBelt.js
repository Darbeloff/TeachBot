function ConveyorBelt() {
	// Initialize key features of the conveyor belt.
	// UL = Upper-Left, it is the starting point of the drawing.
	this.x_UL_belt = 75*cw;
	this.y_UL_belt = 5*ch;
	this.width_belt = 18.0*cw;
	this.length_belt = 90.0*ch;
	this.strap_size = 2*ch;
	this.fillBelt = "#222222";
	this.fillStrap = "#AAAAAA";
	this.fillBox = "#5566CC"
	
	// Create straps on the conveyor belt.
	this.strap_interval = 10*ch;
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
	this.width_box = 12*cw;
	this.length_box = 8*ch;
	this.boxes = []
	this.min_space_boxes = this.y_UL_belt+this.length_box*2;

	// Parameters for pickup area
	this.pickup_x = this.x_UL_belt - this.width_belt*0.15;
	this.pickup_y = this.y_UL_belt + this.length_belt*0.4;
	this.pickup_width = this.width_belt*1.3;
	this.pickup_length = this.length_belt*0.2;
	this.center_y_pickup = this.pickup_y + this.pickup_length/2;
	this.drawPickupArea = false;

	// Temporary variables for testing stuff
	this.temp = this.direction;
	this.count_print = 0;

	self = this;
}

ConveyorBelt.prototype.update = function (timestamp) {

	ctx.clearRect(0,0,100*cw,100*ch);

	this.unit_accel = Math.abs(this.speed_current-this.speed_goal)>0 ? (this.speed_goal-this.speed_current)/Math.abs(this.speed_current-this.speed_goal) : 0;
	this.direction = (this.speed_current == 0) ? 0 : this.speed_current/Math.abs(this.speed_current);
	
	// if (self.count_print>=60) {
	// 	if (self.speed!==0) {
	// 		console.log(self.boxes);
	// 	}
	// 	console.log(this.speed_goal);
	// 	console.log(Math.abs(this.speed_current-this.speed_goal));
	// 	self.count_print = 0;
	// } else { self.count_print++; }

	// Conveyor belt 
	ctx.fillStyle = self.fillBelt;
	ctx.fillRect(self.x_UL_belt, self.y_UL_belt, self.width_belt, self.length_belt);

	// Manipulate speed
	var speed_increment = Math.abs(this.speed_current-this.speed_goal)<0 ? 0 : 0.03*this.unit_accel;
	this.speed_current += speed_increment;
	if (Math.abs(this.speed_current)<0.01) { this.speed_current = 0; }

	// Straps on the belt
	ctx.fillStyle = self.fillStrap;
	self.straps_y.forEach(function (item, index) {
		if (self.direction==1) {
			if (item>=self.strap_begin && item<=self.strap_end) {
				this[index] = item+self.speed_current;
				if (this[index]<=self.strap_end) {
					ctx.fillRect(self.x_UL_belt, this[index], self.width_belt, self.strap_size);
				}
			} else {
				if (item<self.strap_begin) { this[index] = self.strap_end*1.01; }
				if (Math.min(...this)>=self.strap_begin+self.strap_interval && Math.min(...this)>self.strap_begin) {
					this[index] = self.strap_begin;
					ctx.fillRect(self.x_UL_belt, this[index], self.width_belt, self.strap_size);
				}
			}
		} else if (self.direction==-1){
			if (item>=self.strap_begin && item<=self.strap_end) {
				this[index] = item+self.speed_current;
				if (this[index]>=self.strap_begin) {
					ctx.fillRect(self.x_UL_belt, this[index], self.width_belt, self.strap_size);
				}
			} else {
				if (item>self.strap_end) { this[index] = self.strap_begin*0.99; }
				if (Math.max(...this)<=self.strap_end-self.strap_interval && Math.max(...this)<self.strap_end) {
					this[index] = self.strap_end;
					ctx.fillRect(self.x_UL_belt, this[index], self.width_belt, self.strap_size);
				}
			}
		} else {
			if (item>=self.strap_begin && item<=self.strap_end) {
				ctx.fillRect(self.x_UL_belt, this[index], self.width_belt, self.strap_size);
			}
		}
	}, self.straps_y);

	var remove_a_box = false;
	var place_this_box = false;
	self.boxes.forEach(function (item, index) {
		if (index>0){
			if(this[index-1][1]>self.min_space_boxes) {
				this[index][1] += self.speed_current;
				place_this_box = true;
			} else {place_this_box = false;}
		} else {
			this[index][1] += self.speed_current;
			place_this_box = true;
		}
		var outOfBelt = this[index][1]+this[index][3] > (self.y_UL_belt+self.length_belt);
		if (!outOfBelt && place_this_box) {
			ctx.fillStyle = self.fillBox;
			ctx.fillRect(item[0],item[1],item[2],item[3]);
		} else if (outOfBelt) {remove_a_box = true;}
	}, self.boxes);
	// This has to be done outside of boxes.forEach loop to avoid the new 'first' box flickering.
	if (remove_a_box) { self.boxes.splice(0,1); }

	// Draw the pickup region.
	if (this.drawPickupArea) {
		ctx.beginPath();
		ctx.setLineDash([1.0*ch]);
		ctx.lineWidth = 0.3*ch;
		ctx.rect(this.pickup_x,this.pickup_y,this.pickup_width,this.pickup_length);
		ctx.stroke();
	}

	canvas_frame_req = window.requestAnimationFrame(function(timestamp) {
		self.update(timestamp);
	});
}

ConveyorBelt.prototype.stop = function() {
	this.previous_speed = this.speed_current==0 ? this.previous_speed : this.speed_current;
	this.speed_goal = 0;
}

ConveyorBelt.prototype.resume = function() {
	this.speed_goal = this.previous_speed;
}

ConveyorBelt.prototype.changeSpeed = function(amount) {
	this.speed_goal = Math.min(15, Math.max(-15, this.speed_goal+amount));
}

ConveyorBelt.prototype.setSpeed = function(value) {
	this.speed_goal = Math.min(15, Math.max(-15, value));
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
	}
}

ConveyorBelt.prototype.pickupArea = function() {
	this.drawPickupArea = !this.drawPickupArea;
}

ConveyorBelt.prototype.deletePickupArea = function() {
	this.drawPickupArea = false;
}

ConveyorBelt.prototype.create = function() {
	this.something = "hahaha";
}
ConveyorBelt.prototype.show = function() {
	console.log(self.something);
}

// Handled by Module.js, so no need to include in conveyor belt object.
var ctx = myCanvas.getContext("2d");
myCanvas.width = window.innerWidth*0.96;
myCanvas.height = window.innerHeight*0.76;
var ch = myCanvas.height/100.0;
var cw = myCanvas.width/100.0;

var canvas_frame_req;
window.cancelAnimationFrame(canvas_frame_req);

var conveyor = new ConveyorBelt();

canvas_frame_req = window.requestAnimationFrame(function(timestamp) {
	conveyor.update(timestamp);
});

