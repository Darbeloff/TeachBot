function GripperProjection (module_obj) {
	// Basic geometry parameters for constructing the gripper.
	this.base_width = 12*module_obj.cw;
	this.base_length = 4*module_obj.cw;
	this.base_fill = '#222222';
	this.finger_length = 5*module_obj.cw;
	this.finger_thickness = 1*module_obj.cw;
	this.finger_fill = '#E6E6E6';
	this.grip_point = 0;
	this.theta = 0.0;
	this.finger_closed_gap = conveyor.length_box;

	// Variables to check if it is holding a box
	this.box_identified_index = [false, -1];
	this.grabbedBox = false;
	this.box_held_coord = [];
	this.boxes_floor = [];

	// Miscellaneous variables
	this.count_second = 0;
	this.flag_count = true;

	self_GP = this;
}

GripperProjection.prototype.update = async function(module_obj) {

	// Get robot's endpoint info and set up relevant variables for convenience.
	this.robot_xy_canvas = await this.updateEndpoint(module_obj);

	var extension = 5.5*module_obj.cw;
	// Open gripper
	var finger_left_open_x = -this.base_width/2*0.9;
	var finger_right_open_x = this.base_width/2*0.9;
	// Closed gripper
	var finger_left_closed_x = -(this.finger_closed_gap/2+this.finger_thickness);
	var finger_right_closed_x = this.finger_closed_gap/2+this.finger_thickness;
	// Finger's y starting point, which is same regardless of its states.
	var finger_y = this.base_length/2+extension;

	// Calculate the gripper's orientation using attitude and bank
	this.theta = 90 - (-this.attitude); // minum negative works but simple plus does NOT work!
	this.theta *= this.bank/Math.abs(this.bank);

	document.getElementById("Euler").innerHTML = 'Attitude='+this.attitude.toString()+', Bank='+this.bank.toString();

	this.checkBox(module_obj, conveyor);

	// Entering gripper frame.
	module_obj.ctx.save();

	module_obj.ctx.transform(
		Math.cos(this.theta*Math.PI/180),Math.sin(this.theta*Math.PI/180),
		-Math.sin(this.theta*Math.PI/180),Math.cos(this.theta*Math.PI/180),
		this.robot_xy_canvas['x'], this.robot_xy_canvas['y']);

	module_obj.ctx.fillStyle = this.base_fill;
	module_obj.ctx.fillRect(-this.base_width/2, -this.base_length/2+extension, this.base_width,this.base_length);

	module_obj.ctx.fillStyle = this.finger_fill;
	if (this.grip_status=='open') {
		module_obj.ctx.fillRect(finger_left_open_x, finger_y, this.finger_thickness, this.finger_length);
		module_obj.ctx.fillRect(finger_right_open_x, finger_y, -this.finger_thickness, this.finger_length);
		this.grabbedBox = false;
		this.box_held_coord = [];

	} else if (this.grip_status=='closed') {
		module_obj.ctx.fillRect(finger_left_closed_x, finger_y, this.finger_thickness, this.finger_length);
		module_obj.ctx.fillRect(finger_right_closed_x, finger_y, -this.finger_thickness, this.finger_length);
		if (this.grabbedBox) {
			module_obj.ctx.fillStyle = conveyor.fillBox;
			module_obj.ctx.fillRect(this.box_held_coord[0],this.box_held_coord[1],this.box_held_coord[2],this.box_held_coord[3]);
		}

	} else {
		throw 'variable assignment error.';
	}

	// Tip of the gripper in the gripper frame.
	this.grip_point = extension + this.base_length/2 + this.finger_length;
	
	// Back to global frame.
	module_obj.ctx.restore();
}

GripperProjection.prototype.updateEndpoint = async function(module_obj) {
	// Updates robot's raw position info and return by x-y coordinates on the canvas
	module_obj.getEndpoint();
	this.robot_Px = module_obj.dictionary['position_x'];
	this.robot_Py = module_obj.dictionary['position_y'];
	this.robot_Pz = module_obj.dictionary['position_z'];
	var robot_Ox = module_obj.dictionary['orientation_x'];
	var robot_Oy = module_obj.dictionary['orientation_y'];
	var robot_Oz = module_obj.dictionary['orientation_z'];
	var robot_Ow = module_obj.dictionary['orientation_w'];

	var degree = true;
	if (Math.abs(robot_Ox*robot_Oy+robot_Oz*robot_Ow-0.5)<0.00001) {
		this.heading = (2*Math.atan2(robot_Ox, robot_Ow) * (degree?180/Math.PI:1)).toFixed(2);
		this.attitude = (Math.asin(2*robot_Ox*robot_Oy+2*robot_Oz*robot_Ow) * (degree?180/Math.PI:1)).toFixed(2);
		this.bank = 0.0 * (degree?180/Math.PI:1);
	} else if (Math.abs(robot_Ox*robot_Oy+robot_Oz*robot_Ow+0.5)<0.00001) {
		this.heading = (-2*Math.atan2(robot_Ox, robot_Ow) * (degree?180/Math.PI:1)).toFixed(2);
		this.attitude = (Math.asin(2*robot_Ox*robot_Oy+2*robot_Oz*robot_Ow) * (degree?180/Math.PI:1)).toFixed(2);
		this.bank = 0.0 * (degree?180/Math.PI:1);
	} else {
		this.heading = (Math.atan2(2*robot_Oy*robot_Ow-2*robot_Ox*robot_Oz, 1-2*Math.pow(robot_Oy,2)-2*Math.pow(robot_Oz,2)) * (degree?180/Math.PI:1)).toFixed(2);
		this.attitude = (Math.asin(2*robot_Ox*robot_Oy+2*robot_Oz*robot_Ow).toFixed(2) * (degree?180/Math.PI:1)).toFixed(2);
		this.bank = (Math.atan2(2*robot_Ox*robot_Ow-2*robot_Oy*robot_Oz, 1-2*Math.pow(robot_Ox,2)-2*Math.pow(robot_Oz,2)) * (degree?180/Math.PI:1)).toFixed(2);
	}

	if (this.robot_Px==null) {
		return NaN;
	} else {
		return module_obj.robot2canvas(this.robot_Px, this.robot_Py).then(function(res) {return res;});
	}
}

GripperProjection.prototype.checkBox = function(module_obj, conveyor_belt) {
	this.box_identified_index = [false, -1];
	var extension = 5.5*module_obj.cw;

	// Grip point in the global frame, not the gripper frame which has its origin at near-wrist position.
	var grip_orig_x = Math.cos(-this.theta*Math.PI/180)*0 + Math.sin(-this.theta*Math.PI/180)*this.grip_point + this.robot_xy_canvas['x'];
	var grip_orig_y = -Math.sin(-this.theta*Math.PI/180)*0 + Math.cos(-this.theta*Math.PI/180)*this.grip_point + this.robot_xy_canvas['y'];
	module_obj.ctx.beginPath();
	module_obj.ctx.arc(grip_orig_x,grip_orig_y,2,0,2*Math.PI);
	module_obj.ctx.fillStyle = 'green';
	module_obj.ctx.fill();
	module_obj.ctx.stroke();

	if (this.count_second>60 && this.flag_count==true) {
		// console.log(grip_orig_x);
		this.count_second = 0;
	} else { if (this.flag_count){this.count_second++;} }

	if (grip_orig_x!==NaN && grip_orig_y!==NaN) {
		conveyor_belt.boxes.forEach(function (item, index) {
			if (grip_orig_y>item[1] && grip_orig_y<item[1]+item[3]) {
				if (grip_orig_x-item[0]<self_GP.finger_length && grip_orig_x-item[0]>0) {
					self_GP.box_identified_index[0] = true;
					self_GP.box_identified_index[1] = index;
					if (!self_GP.grabbedBox) {
						self_GP.box_x_gripper_frame = -item[3]/2;
						self_GP.box_y_gripper_frame = self_GP.grip_point-(grip_orig_x-item[0]);
						console.log(self_GP.box_y_gripper_frame)
					}
					if (self_GP.grip_status=='closed') {
						conveyor.boxes.splice(index,1);
						conveyor.boxes_Ycenter.splice(index,1);
						self_GP.box_held_coord = [self_GP.box_x_gripper_frame,self_GP.box_y_gripper_frame,item[3],item[2]];
						self_GP.grabbedBox = true;
					} else {
						self_GP.grabbedBox = false;
					}
				} else {
					console.log('DEBUG; conditions for pickup not met.')
				}
			} else {
				console.log('DEBUG; conditions for pickup not met.')
			}
		});
	} else {
		console.log('One of them is NaN at least.')
	}
}

GripperProjection.prototype.GripperStatus = function(signal) {
	// signal[0].data returns '[true]' if the gripper is closed.
	// signal[1].data returns '[true]' if the gripper is closed (but is called grip, why tho...).
	// signal[2].data returns '[true]' if the gripper is open.
	this.grip_status = signal[0].data=='[true]' ? 'closed' : 'open';
	console.log('Gripper is ' + this.grip_status);
}

function openGripper(module_obj) {
	var goal = new ROSLIB.Goal({
		actionClient: module_obj.GripperAct,
		goalMessage:{
			grip:false
		}
	});
	goal.send();
}

function closeGripper(module_obj) {
	var goal = new ROSLIB.Goal({
		actionClient: module_obj.GripperAct,
		goalMessage:{
			grip:true
		}
	});
	goal.send();
}

// Both for testing only. Will be deprecated.
function enableCuffInteraction() {
	var goal = new ROSLIB.Goal({
		actionClient: m.AllowCuffInteractionAct,
		goalMessage:{
			allow:true
		}
	});
	goal.on('result', function(result) {
		console.log('enabled');
	});
	goal.send();
}

function disableCuffInteraction() {
	var goal = new ROSLIB.Goal({
		actionClient: m.AllowCuffInteractionAct,
		goalMessage:{
			allow:false
		}
	});
	goal.on('result', function(result) {
		console.log('disabled');
	});
	goal.send();
}

var GP = new GripperProjection(m);