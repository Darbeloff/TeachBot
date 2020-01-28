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
	
	// Create straps on the conveyor belt.
	this.strap_interval = 10*ch;
	this.straps_y = [];
	for (var i=this.y_UL_belt;i<this.length_belt;i+=this.strap_interval) {
		this.straps_y.push(i);
	}
	this.strap_begin = this.y_UL_belt;
	this.strap_end = this.strap_begin + this.length_belt - this.strap_size;

	// Status variables
	this.speed = 0;
	this.speed_factor = 5;
	this.previous_speed = this.speed;
	this.direction = 0;

	// Boxes on top of the conveyor belt.
	this.width_box = 12*cw;
	this.length_box = 8*ch;
	this.boxes = []

	// Temporary variables for testing stuff
	this.temp = this.direction;
	this.count_print = 0;

	self = this;
}

ConveyorBelt.prototype.update = function (timestamp) {

	ctx.clearRect(0,0,100*cw,100*ch);

	self.direction = (self.speed == 0) ? 0 : self.speed/Math.abs(self.speed);
	if (self.temp !== self.direction) {
		console.log('Direction is '+ self.direction);
		self.temp = self.direction;
	}

	if (self.count_print==60) {
		if (self.speed!==0) {
			console.log(self.straps_y);
		}
		self.count_print = 0;
	} else { self.count_print++; }

	// Conveyor belt
	ctx.fillStyle = self.fillBelt;
	ctx.fillRect(self.x_UL_belt, self.y_UL_belt, self.width_belt, self.length_belt);

	// Straps on the belt
	ctx.fillStyle = self.fillStrap;
	self.straps_y.forEach(function (item, index) {
		if (self.direction==1) {
			if (item>=self.strap_begin && item<=self.strap_end) {
				this[index] = item+self.speed/self.speed_factor;
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
				this[index] = item+self.speed/self.speed_factor;
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

	self.boxes.forEach(function (item, index) {

	}, self.boxes);

	canvas_frame_req = window.requestAnimationFrame(function(timestamp) {
		self.update(timestamp);
	});
}

ConveyorBelt.prototype.stop = function() {
	this.previous_speed = this.speed==0 ? this.previous_speed : this.speed;
	this.speed = 0;
}

ConveyorBelt.prototype.resume = function() {
	this.speed = this.previous_speed;
}

ConveyorBelt.prototype.changeSpeed = function(amount) {
	this.speed = Math.min(15, Math.max(-15, this.speed+amount));
}

ConveyorBelt.prototype.setSpeed = function(value) {
	this.speed = Math.min(10, Math.max(-10, value));
}

ConveyorBelt.prototype.addBox = function() {

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

