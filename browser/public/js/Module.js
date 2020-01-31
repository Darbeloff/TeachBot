// Constants
const DIR = 'https://localhost:8000/';    // Directory containing resources
const JOINTS = 7;                         // Numer of joints in Sawyer arm
const VERBOSE = true;                     // Whether or not to print everything
const BUTTON = {'back': 0, 'show': 1, 'circle': 2, 'square': 3, 'triangle': 4};
const ROBOT = 'sawyer';

/**
 * A learning module for TeachBot.
 *
 * General interpreter for module JSON files. Loads, parses, and plays instructions.
 * Example of how to use:
 * >> var m = new Module(1, main, [image,canvas_container]);
 * >> async function main() {
 * >>     m.displayOff();
 * >>     image.style.display = 'initial';
 * >>     m.start();
 * >> }
 *
 * @class
 *
 * @param {number}		module_num			The module number. Used to find JSON, audio, and text files.
 * @param {function}	main 				Function to run when all resources have been loaded.
 * @param {Array}		content_elements	List of all HTML elements that have display properties.
 */
function Module(module_num, main, content_elements) {
	if (VERBOSE) console.log(`Beginning Module ${module_num}`);

	// Instance Variables
	this.module_num = module_num;											// The index of the current module. TODO: replace with Claire's naming structure
	this.main = main;														// The function to run after all resources have been loaded.
	this.content_elements = content_elements;								// The elements to be loaded. TODO: consider deprecating per Tongxi's suggestion.
	this.loaded = {'audio':false, 'text':false, 'json':false};				// Dictionary to track which resources have loaded.
	this.drawings = [];														// Array of objects to draw on the canvas.
	this.graphic_mode = 'image';											// Current graphic mode. See: set_graphic_mode().
	this.canvas_frame_req;													// Animation frame request. See: set_graphic_mode().
	
	// Initialize self to module for use in event callbacks
	self = this;

	/*******************************
	 *   Setup ROS communication   *
	 *******************************/
	var ros = new ROSLIB.Ros({ url: 'wss://localhost:9090' });
	ros.on('connection', function() { console.log('Connected to websocket server.'); });
	ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); window.alert('Error connecting to websocket server'); });
	ros.on('close', function() { console.log('Connection to websocket server closed.'); });

	// Publishing topics
	this.camera = new ROSLIB.Topic({
		ros: ros,
		name: '/teachbot/camera',
		messageType: 'std_msgs/Bool'
	})

	// Subscribing topics
	this.command_complete = new ROSLIB.Topic({
		// Deprecated, but still used by 'camera' and 'camera_off'
		ros: ros,
		name: '/teachbot/command_complete',
		messageType: 'std_msgs/Empty'
	})
	this.wheel_delta_topic = new ROSLIB.Topic({
		ros: ros,
		name: '/teachbot/wheel_delta',
		messageType: 'std_msgs/Int32'
	})
	this.scroll_wheel_button_receiver = new ROSLIB.Topic({
		// Deprecated, but still used by 'scrollWheelInput'
		ros: ros,
		name: '/teachbot/scroll_wheel_button_topic',
		messageType: 'std_msgs/Empty'
	})
	this.pressed = new ROSLIB.Topic({
		// Deprecated, but still used by 'drawDynamic', 'interaction_projection', 'pressed_button', 'projection', and 'scrollWheelInput'
		ros: ros,
		name: '/teachbot/scroll_wheel_pressed',
		messageType: 'std_msgs/Bool'
	});

	this.position = new ROSLIB.Topic({
		ros: ros,
		name: '/teachbot/position',
		messageType: ROBOT + '/JointInfo'
	});
	this.position.subscribe(self.positionCallback);
	this.velocity = new ROSLIB.Topic({
		ros: ros,
		name: '/teachbot/velocity',
		messageType: ROBOT + '/JointInfo'
	});
	this.velocity.subscribe(self.velocityCallback);
	this.effort = new ROSLIB.Topic({
		ros: ros,
		name: '/teachbot/effort',
		messageType: ROBOT + '/JointInfo'
	});
	this.effort.subscribe(self.effortCallback);
	this.endpoint = new ROSLIB.Topic({
		ros: ros,
		name: '/teachbot/EndpointInfo',
		messageType: ROBOT + '/EndpointInfo'
	});
	this.endpoint.subscribe(self.endpointCallback);

	// Service Servers
	var DevModeSrv = new ROSLIB.Service({
		ros: ros,
		name: '/teachbot/dev_mode',
		serviceType: ROBOT + '/DevMode'
	});
	DevModeSrv.advertise(this.devRxCallback);

	// Service Clients
	this.SetRobotModeSrv = new ROSLIB.Service({
		ros: ros,
		name: '/teachbot/set_robot_mode',
		serviceType: 'SetRobotMode'
	});
	this.UpdateAudioDurationSrv = new ROSLIB.Service({
		ros: ros,
		name: '/teachbot/audio_duration',
		serviceType: 'AudioDuration'
	});
	this.ScrollWheelSubscriptionSrv = new ROSLIB.Service({
		ros: ros,
		name: '/teachbot/wheel_subscription',
		serviceType: 'ScrollWheelSubscription'
	});

	// Actions
	this.CuffInteractionAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/CuffInteraction',
		actionName: ROBOT + '/CuffInteractionAction'
	});
	this.GoToJointAnglesAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/GoToJointAngles',
		actionName: ROBOT + '/GoToJointAnglesAction'
	});
	this.JointMoveAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/JointMove',
		actionName: ROBOT + '/JointMoveAction'
	});
	this.InteractionControlAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/InteractionControl',
		actionName: ROBOT + '/InteractionControlAction'
	});
	this.AdjustPoseToAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/AdjustPoseTo',
		actionName: ROBOT + '/AdjustPoseToAction'
	});
	this.GripperAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/Gripper',
		actionName: ROBOT + '/GripperAction'
	});
	this.GoToCartesianPoseAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/GoToCartesianPose',
		actionName: ROBOT + '/GoToCartesianPoseAction'
	});
	this.MultipleChoiceAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/MultipleChoice',
		actionName: ROBOT + '/MultipleChoiceAction'
	});
	this.PickUpBoxAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/PickUpBox',
		actionName: ROBOT + '/PickUpBoxAction'
	});
	this.AdjustPoseByAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/AdjustPoseBy',
		actionName: ROBOT + '/AdjustPoseByAction'
	});
	this.JointImpedanceAct = new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/JointImpedance',
		actionName: ROBOT + '/JointImpedanceAction'
	});
	this.WaitAct =  new ROSLIB.ActionClient({
		ros: ros,
		serverName: '/teachbot/Wait',
		actionName: ROBOT + '/WaitAction'
	})

	// Initialize dictionary
	this.dictionary = {};
	this.getEndpoint();

	/*********************
	 *   HTML Elements   *
	 *********************/
	this.ctx = canvas_obj.getContext('2d');
	canvas_obj.width = window.innerWidth*0.96;
	canvas_obj.height = window.innerHeight*0.76;
	this.ch = canvas_obj.height/100.0;
	this.cw = canvas_obj.width/100.0;
	this.robot_color = getComputedStyle(document.body).getPropertyValue('--robot-color')

	/************************
	 *   Update Font Size   *
	 ************************/
	this.textBoxMaxHeight = document.getElementById("adjustable").scrollHeight;

	/******************************
	 *   Setup Section Sequence   *
	 ******************************/
	var jsonPath = DIR + 'js/json/' + this.module_num + '.json';
	var jqhxr = $.getJSON(jsonPath, function(data) {
		self.sections = data.sections;
		for (let s=0; s<self.sections.length; s++) {
			self.sections[s]._textLoaded = false;
			self.sections[s]._audioLoaded = false;
		}

		self.loadTextAndAudio();
	});
	jqhxr.fail(function() {
		throw new Error('JSON file not formatted correctly. Go to ' + jsonPath + ' to learn more.')
	});
	jqhxr.done(function() {
		self.loaded['json'] = true;
		if (self.allLoaded()) { self.main(); }
	});
}

// Callbacks
Module.prototype.positionCallback = function(msg) {
	for (let j=0; j<Object.keys(msg).length; j++) {
		self.dictionary[`JOINT_POSITION_${j}`] = msg[`j${j}`];
	}
}
Module.prototype.velocityCallback = function(msg) {
	for (let j=0; j<Object.keys(msg).length; j++) {
		self.dictionary[`JOINT_VELOCITY_${j}`] = msg[`j${j}`];
	}
}
Module.prototype.effortCallback = function(msg) {
	for (let j=0; j<Object.keys(msg).length; j++) {
		self.dictionary[`EFFORT_${j}`] = msg[`j${j}`];
	}
}
Module.prototype.endpointCallback = function(msg) {
	self.dictionary[`ENDPOINT_POSITION_X`] = msg.position.x;
	self.dictionary[`ENDPOINT_POSITION_Y`] = msg.position.y;
	self.dictionary[`ENDPOINT_POSITION_Z`] = msg.position.z;
	self.dictionary[`ENDPOINT_ORIENTATION_X`] = msg.orientation.x;
	self.dictionary[`ENDPOINT_ORIENTATION_Y`] = msg.orientation.y;
	self.dictionary[`ENDPOINT_ORIENTATION_Z`] = msg.orientation.z;
	self.dictionary[`ENDPOINT_ORIENTATION_W`] = msg.orientation.w;
}

Module.prototype.unsubscribeFrom = function(topic) {
	topic.unsubscribe();
	topic.removeAllListeners();
	switch (topic.name) {
		case '/teachbot/position':
			topic.subscribe(self.positionCallback);
			break;
		case '/teachbot/velocity':
			topic.subscribe(self.velocityCallback);
			break;
		case '/teachbot/effort':
			topic.subscribe(self.effortCallback);
			break;
		case '/teachbot/endpoint':
			topic.subscribe(self.endpointCallback);
			break;
	}
}

/**
 * Loads text and audio.
 */
Module.prototype.loadTextAndAudio = function() {
	// Base of directory containing text
	this.text_dir = DIR + 'text/module' + this.module_num + '/';
	
	// For each section
	for (let s=0; s<this.sections.length; s++) {

		// Load all lines in corresponding text file.
		jQuery.get(this.text_dir + this.sections[s].id + '.txt', function(data) {

			// Load text.
			self.sections[s]._textArray = data.split('\n');
			self.sections[s]._textLoaded = true;

			// Load audio.
			var audioCount = self.sections[s]._textArray.length;

			// If there are no audio files in this section, mark the section as loaded.
			if (audioCount==0) {
				self.sections[s]._textLoaded = true;
				self.sections[s]._audioLoaded = true;

			// Otherwise, load them.
			} else {
				self.sections[s]._audiofiles_mp3 = new Array(audioCount);
				self.sections[s]._audio_duration = new Array(audioCount);
				self.sections[s]._audio_duration_copy = new Array(audioCount);
				self.sections[s]._num_loaded = 0;
				for (let a=0; a<audioCount; a++) {
					self.sections[s]._audiofiles_mp3[a] = DIR + 'audio/module' + self.module_num + '/' + self.sections[s].id + '/line' + a.toString() + '.mp3';
					let audio = new Audio();
					audio.addEventListener('canplaythrough', function() {
						self.sections[s]._audio_duration[a] = audio.duration*1000;
						self.sections[s]._audio_duration_copy[a] = audio.duration*1000;
						self.sections[s]._num_loaded++;
						if (self.sections[s]._num_loaded === audioCount) {
							self.sections[s]._audioLoaded = true;

							for (let ss=0; ss<self.sections.length; ss++) {
								if (!self.sections[ss]._audioLoaded || !self.sections[ss]._textLoaded) {
									return;
								}
							}
							self.loaded['text'] = true;
							self.loaded['audio'] = true;
							if (self.allLoaded()) { self.main(); }
						}
					}, false);
					audio.src = self.sections[s]._audiofiles_mp3[a];
				}
			}

			// If all the sections' text and audio files have been loaded, mark it and check to proceed.
			for (let ss=0; ss<self.sections.length; ss++) {
				if (!self.sections[ss]._audioLoaded || !self.sections[ss]._textLoaded) {
					return;
				}
			}
			self.loaded['text'] = true;
			self.loaded['audio'] = true;
			if (self.allLoaded()) { self.main(); }
		});
	}
}

/**
 * Play audio file.
 *
 * Plays a given audio file.
 *
 * @param {string}	audioFile 	URL of audio file.
 * @param {number}	duration 	Duration of audio file in milliseconds.
 * @param {string} 	text 		Text to display on screen while playing audio.
 */
Module.prototype.play = async function(audioFile, duration, text) {
	// Play audio.
	player.src = audioFile;
	player.play();

	// Update text on screen.
	adjustable.innerHTML = text;
	this.adjust_text();

	// Convert units of duration from [ms]->[s]
	duration/=1000.0;

	// Inform robot for how long current audio file will play.
	this.UpdateAudioDurationSrv.callService(new ROSLIB.ServiceRequest({audio_duration: duration}), result => {return});

	// Log action.
	if (VERBOSE) console.log('Playing ' + audioFile + ', duration ' + Math.round(duration) + 's.');
}

/**
 * Format a GoToJointAngles goal for sending.
 *
 * When the returned object is sent, the robot should move to the position specified.
 *
 * @param  {[string,Array]}	joint_angles 	Either an Array of joint angles or the name of the specific pose.
 * @param  {number}		    [speed_ratio=0]	Fraction of maximum speed for the robot to move.
 * @param  {bool}           [wait=false]    Tells the robot whether or not to wait for the audio file to be done playing.
 * @return {object}                         ROSLIB.Goal object to be sent.
 */
Module.prototype.getGoToGoal = function(joint_angles, speed_ratio=0, wait=false) {
	var goalMessage = {
		speed_ratio: speed_ratio,
		wait: wait
	}

	if (typeof joint_angles === 'string') {
		goalMessage.name = this.hashTokeyVal(joint_angles);
	} else if (joint_angles instanceof Array) {
		for (let j=0; j<joint_angles.length; j++) {
			goalMessage[`j${j}pos`] = joint_angles[j];
		}
	} else {
		throw 'goToJointAngles instruction incorrectly formatted.';
	}

	return new ROSLIB.Goal({
		actionClient: self.GoToJointAnglesAct,
		goalMessage: goalMessage
	});
}

/**
 * Format JointMove goal for sending.
 *
 * When the returned object is sent, the robot should activate admittance control.
 *
 * @param  {[string,Array]}	joints 	             Either an Array of joint(s) that determines which joints to unlock.
 * @param  {string}			terminatingCondition Condition that once reached, the program exit out of the function.
 * @param  {string}         resetPOS             String that states the position the robot should default to after the terminatingCondition is reached
 * @param  {string}         min_thresh           Specifies the minimum amount of force that requires the joint(s) to move
 * @param  {string}         bias                 Each joint has a bias that takes into account an internal force in the robot and counteracts it
 * @param  {float32}        tol                  Float that is used in the callback function in the teachbot script
 * @return {object}                              ROSLIB.Goal object to be sent.
 */
Module.prototype.getJointMoveGoal = function(joints, terminatingCondition, resetPOS, min_thresh, bias, tol = 0) {
	return new ROSLIB.Goal({
		actionClient: self.JointMoveAct,
		goalMessage: {
			joints: self.hashTokeyVal(joints),
			terminatingCondition: self.hashTokeyVal(terminatingCondition),
			resetPOS: self.hashTokeyVal(resetPOS),
			min_thresh: self.hashTokeyVal(min_thresh),
			bias: self.hashTokeyVal(bias),
			tol: tol
		}
	});
}

Module.prototype.pub_angle = function(angle) {
	
	var req = new ROSLIB.Message({
		data: this.hashTokeyVal(angle)
	});

	this.joint_angle.publish(req);

}

Module.prototype.getEndpoint = function(){
	this.endpoint.subscribe(function(message) { 
		self.dictionary['position_x'] = message.position.x;
		self.dictionary['position_y'] = message.position.y;
		self.dictionary['position_z'] = message.position.z;
		self.dictionary['orientation_x'] = message.orientation.x;
		self.dictionary['orientation_y'] = message.orientation.y;
		self.dictionary['orientation_z'] = message.orientation.z;
		self.dictionary['orientation_w'] = message.orientation.w;
		//console.log(self.dictionary)	
	});
}


/**
 * Adjusts the text size to fit in the text box.
 *
 * Adjusts the font until the text fills the most space possible in the text box or the maximum font size is reached.
 *
 * @param {string}	[max_font_size='64px']	The maximum allowable font size.
 */
Module.prototype.adjust_text = function(max_font_size='64px') {
	var resize_me = document.getElementById('adjustable');
	resize_me.style.fontSize = max_font_size;
	while (resize_me.scrollHeight > this.textBoxMaxHeight && parseInt(resize_me.style.fontSize)>2){
		resize_me.style.fontSize = parseInt(resize_me.style.fontSize) - 1 + 'px';
	}
}

/**
 * Whether all page resources are loaded.
 * 
 * Returns true iff all the values in self.loaded are true.
 */
Module.prototype.allLoaded = function() {
	for (let key in self.loaded) {
		if (!self.loaded[key]) {
			return false;
		}
	}
	return true;
}

/**
 * Hides all elements on page.
 *
 * Turns the 'display' property of all elements in self.content_elements to 'none'.
 * Optionally also clears the canvas.
 *
 * @param {boolean}	[clearCanvas=false]	Whether or not to also clear the entire canvas.
 */
Module.prototype.displayOff = function(clearCanvas=false) {
	this.content_elements.forEach(function(elem) {
		elem.style.display = 'none';
	});
	if (clearCanvas) {
		this.drawings = [];
		this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
	}
}

/**
 * Toggles development mode.
 *
 * Callback for the DevModeSrv service.
 * Turns development mode on if the message is 1, off if 0.
 *
 * @param {object}	req 	ROS message containing 0 or 1.
 * @param {object}  resp    To be honest, I don't really know why this is an input.
 */
Module.prototype.devRxCallback = function(req, resp) {
	switch (req.status) {
		case 0:
			self.devMode = false
			for (let s=0; s<self.sections.length; s++) {
				self.sections[s]._audio_duration = self.sections[s]._audio_duration_copy.slice();
			}
			if (VERBOSE) console.log('Exiting dev mode.');
			break;

		case 1:
			self.devMode = true
			for (let s=0; s<self.sections.length; s++) {
				for (let d=0; d<self.sections[s]._audio_duration.length; d++) {
				self.sections[s]._audio_duration[d] = 0;
				}
			}
			if (VERBOSE) console.log('Entering dev mode.');
			break;
	}

	resp['success'] = true;
    resp['message'] = 'Set successfully';
    return true;
}

/**
 * Coordinate transform from robot-space to 
 */
Module.prototype.robot2canvas = async function(x, y, robot='sawyer') {
	const Kx = 72.73;
	const bx = 51.45;
	const Ky = 157.89;
	const by = -30;

	switch (robot) {
		case 'sawyer':
			x_canvas = (Kx * y + bx) * this.cw;
			y_canvas = (Ky * x + by) * this.ch;

		default:
			throw `Robot ${robot} not supported.`;
	} 

	return {
		x: x_canvas,
		y: y_canvas
	};
}

/**
 * Runs a command from the JSON file.
 *
 * Finds a command from a given address in the JSON file and performs it, then advances to the next command.
 * The command address should be formatted as an array with the following elements:
 *   0: {string}  Name of section.
 *   1: {number}  Index of instruction within section.
 *   The following elements are only valid if the instruction is of type 'if' or 'while', and may proceed indefinitely:
 *     If the parent instruction is of type 'if':
 *       n  : {boolean} Whether to follow the 'if_true' or 'if_false' instructions next.
 *       n+1: {number}  Index of the child instruction.
 *     Else if the parent instruction is of type 'while':
 *       n  : {number}  Index of the child instruction.
 *
 * @param {Array}	[instructionAddr=['intro',0]]	Address of instruction to play.
 */
Module.prototype.start = async function(instructionAddr=['intro',0]) {
	this.thisSection;
	for (let s=0; s<this.sections.length; s++) {
		if (this.sections[s].id === instructionAddr[0]) {
			this.thisSection = this.sections[s];
			break;
		}
	}
	if (this.thisSection === undefined) {
		throw `There is no section with id ${instructionAddr[0]}`;
	}
 
	var instructions = this.thisSection.instructions;
	this.instructionSets = [JSONcopy(instructions)];

	if (instructionAddr.length == 1) {
		console.log('Reached End of Module.');
		return;
	}

	var instr = instructions[instructionAddr[1]];
	for (let instructionAddrLevel=2; instructionAddrLevel<instructionAddr.length; instructionAddrLevel++) {
		if (instr.type === 'if') {
			instructionAddrLevel++;
			if (instructionAddr[instructionAddrLevel-1]) {
				instructions = instr.if_true;
			} else {
				instructions = instr.if_false;
			}
		} else if (instr.type === 'while') {
			instructions = instr.instructions;
		} else {
			throw `Compound instruction addressing not available for instructions of type ${instr.type}`;
		}
		this.instructionSets.push(JSONcopy(instructions));
		instr = instructions[instructionAddr[instructionAddrLevel]];
	}

	if (instr.hasOwnProperty('skippable') && instr.skippable && self.devMode) {
		this.start(this.getNextAddress(instructionAddr));
	} else {
		switch(instr.type) {
			case 'pause':

				console.log("module paused here.")

				break;

			// case 'shadow':

			// 	const Kx = 72.73;
			// 	const bx = 51.45;
			// 	const Ky = 157.89;
			// 	const by = -30;

			// 	this.endpoint.subscribe(async function(message) {
			// 		self.ctx.clearRect(0,0,100*self.cw,100*self.ch);
			// 		var x_center = Kx * message.position.y + bx;
			// 		var y_center = Ky * message.position.x + by;
			// 		draw_ball(self.ctx, x_center, y_center, 8, '#7c2629');
			// 	});

			// 	this.pressed.subscribe(async function(message) {
			// 		if (VERBOSE) console.log('Pressed: ' + message.data);
			// 		if (message.data == true) {
			// 			self.endpoint.unsubscribe();
			// 			self.endpoint.removeAllListeners();
			// 			self.pressed.unsubscribe();
			// 			self.pressed.removeAllListeners();
			// 			self.displayOff();
			// 			self.start(self.getNextAddress(instructionAddr));
			// 		}
			// 	});

			// 	break

			case 'adjustPoseBy':
				checkInstruction(instr, ['geometry', 'axis', 'amount'], instructionAddr);

				var goal_AdjustPoseBy = new ROSLIB.Goal({
					actionClient: this.AdjustPoseByAct,
					goalMessage: {
						geometry: this.hashTokeyVal(instr.geometry),
						axis: this.hashTokeyVal(instr.axis),
						amount: this.hashTokeyVal(instr.amount)
					}
				});
				goal_AdjustPoseBy.on('result', function(result){
					self.start(self.getNextAddress(instructionAddr));
				});
				goal_AdjustPoseBy.send()

				break;

			case 'adjustPoseTo':
				checkInstruction(instr, ['geometry', 'axis', 'amount'], instructionAddr);

				var goal_AdjustPoseTo = new ROSLIB.Goal({
					actionClient: this.AdjustPoseToAct,
					goalMessage:{
						geometry: this.hashTokeyVal(instr.geometry),
						axis: this.hashTokeyVal(instr.axis),
						amount: this.hashTokeyVal(instr.amount)
					}
				});
				goal_AdjustPoseTo.on('result', function(result){
					self.start(self.getNextAddress(instructionAddr));
				});
				goal_AdjustPoseTo.send()

				break;

			case 'initializeDisplay':
				this.displayOff();
				canvas_container.style.display = 'initial';
				this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
				this.start(self.getNextAddress(instructionAddr));
				break;

			case 'draw':
				this.draw(instr, instructionAddr);
				break;
/*
			case 'drawShape':
				checkInstruction(instr, ['shape'], instructionAddr);

				if (instr.shape=='ball') {
					if (instr.hasOwnProperty('clearRec')){this.ctx.clearRect(0,0,100*this.cw,100*this.ch);};
					var x_ball = instr.x_ratio*this.cw;
					var y_ball = instr.y_ratio*this.ch;
					var r_ball = instr.r_ratio*this.ch;
					if (instr.hasOwnProperty('label')) {
						this.ctx.font = Math.round(instr.labelsize_ratio*this.cw) + 'px Raleway';
						draw_ball(this.ctx, x_ball, y_ball, r_ball, instr.fillStyle, instr.label);
					} else{
						draw_ball(this.ctx, x_ball, y_ball, r_ball, instr.fillStyle);
					}
				} else if (instr.shape=='arc') {
					canvas_container.style.display = 'initial';
					arc3pt(this.ctx,instr.x1*this.cw,instr.y1*this.ch,instr.x2*this.cw,instr.y2*this.ch,instr.x3*this.cw,instr.y3*this.ch,instr.ccw);
				} else if (instr.shape=='bar') {
					draw_bar_new(this.ctx, instr.x_ratio*this.cw, instr.y_ratio*this.ch, instr.width_ratio*this.cw, instr.max_height_ratio*this.ch, instr.height_percent, instr.fillStyle, instr.label);
				} else if (instr.shape=='rectangle') {
					if (instr.hasOwnProperty('label')){
						draw_rectangle(this.ctx, instr.x_ratio*this.cw, instr.y_ratio*this.ch, instr.w_ratio*this.cw, instr.h_ratio*this.ch, instr.rotate, instr.label)
					} else {
						draw_rectangle(this.ctx, instr.x_ratio*this.cw, instr.y_ratio*this.ch, instr.w_ratio*this.cw, instr.h_ratio*this.ch, instr.rotate)
					}
				}
				this.start(self.getNextAddress(instructionAddr));

				break;
*/
			case 'LOG':
				console.log(Object.keys(instr.aDict))
				for (var key in (instr.aDict)) {
					console.log('individual key: '+key)
				}

				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'draw_dynamic':
				

				for (var topic in instr.topics) {				// Loop through all topics.
					var values = instr.topics[topic];
					if (topic == 'position') {					// Check what type of topics it is.
						this.position.subscribe(async function(message) {
							if (instr.shape == 'bar') {
								/** 
								 *  The line below needs to be fixed.
								 *  When multiple topics are fed, it clears previous topics.
								 *  Not an issue for now.
								 */
								self.ctx.clearRect(0,0,100*self.cw,100*self.ch);
								self.ctx.font = Math.round(3*self.cw) + 'px Raleway';
								for (let i=0;i<values.length;i++){		// Loop through every element and update the graph
									var joint = values[i]
									if (VERBOSE) console.log('Received: ' + joint + ': ' + eval('message.'+joint));
									var x = instr.x_ratio[i]*self.cw;
									var y = instr.y_ratio[i]*self.ch;
									var width = instr.width_ratio[i]*self.cw;
									var max_height = instr.max_height_ratio[i]*self.ch;
									var height_percent;
									if (joint == 'j6') {
										height_percent = (eval('message.'+joint)+1.5*Math.PI)/(3*Math.PI);
									} else {
										height_percent = (eval('message.'+joint)+Math.PI)/(2*Math.PI);
									}
									if (instr.hasOwnProperty('label')) {
										draw_bar_new(self.ctx, x, y, width, max_height, height_percent, instr.fillStyle[i], instr.label[i]);
									} else{
										draw_bar_new(self.ctx, x, y, width, max_height, height_percent, instr.fillStyle[i]);
									};
								};
							} else if (instr.shape == 'ball'){
								// Placeholder, doing nothing for now.
							};
						});
					} else if (topic == 'velocity'){
						// placeholder for another topic. Also serves as an example of what a topic can be.
					}
				};

				this.pressed.subscribe(async function(message) {
					if (VERBOSE) console.log('Pressed: ' + message.data);
					if (message.data == true) {
						for (topic in instr.topics){
							eval('self.'+topic+'.unsubscribe();');
							eval('self.'+topic+'.removeAllListeners();');
						}
						self.pressed.unsubscribe();
						self.pressed.removeAllListeners();
						self.displayOff();
						self.start(self.getNextAddress(instructionAddr));
					}
				});

				break;

			case 'camera':
				var req = new ROSLIB.Message({
					data: true
				});

				console.log('Getting camera info')

				this.camera.publish(req);

				this.command_complete.subscribe(function(message) {
					self.command_complete.unsubscribe();
					self.command_complete.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

				break;

			case 'camerabw_graphic':

				var cv_image_url = DIR + 'images/cv_1.png';

				image.src = cv_image_url;
				
				this.start(this.getNextAddress(instructionAddr));

				break;

			case 'camera_color_graphic':
				var cv_image_url = DIR + 'images/cv_2.png';

				image.src = cv_image_url;
				
				this.start(this.getNextAddress(instructionAddr));

				break;

			case 'camera_off':
				var req = new ROSLIB.Message({
					data: false
				});

				console.log('Shutting down camera')

				this.camera.publish(req);

				this.command_complete.subscribe(function(message) {
					self.command_complete.unsubscribe();
					self.command_complete.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

				break;

			case 'check_pickup':

				var goal_PickUpBox = new ROSLIB.Goal({
					actionClient: this.PickUpBoxAct,
					goalMessage:{check: true}
				});
				goal_PickUpBox.on('result', function(result){
					if (VERBOSE) console.log('Picked up: ' + result);
					if (result.is_picked == true) {
						wheel_val = 1
						if (instr.hasOwnProperty('store_answer_in')) {
							self.dictionary[instr.store_answer_in] = wheel_val;
						}
						self.start(self.getNextAddress(instructionAddr));
					} else{
						wheel_val = 2
						if (instr.hasOwnProperty('store_answer_in')) {
							self.dictionary[instr.store_answer_in] = wheel_val;
						}
						self.start(self.getNextAddress(instructionAddr));
					}
				});
				goal_PickUpBox.send();

				break;

			case 'clearRect':
				this.ctx.clearRect(0,0,100*m.cw,100*m.ch);
				
				this.start(this.getNextAddress(instructionAddr));

				break;

			case 'cuff_interaction':
				checkInstruction(instr, ['terminatingCondition', 'ways'], instructionAddr);

				var orient_bw_url = DIR + 'images/orientation_bw.png';
				var orient_color_url = DIR + 'images/orientation_color.png';
				var position_bw_url = DIR + 'images/position_bw.png';
				var position_color_url = DIR + 'images/position_color.png';

				var goal = new ROSLIB.Goal({
					actionClient: this.CuffInteractionAct,
					goalMessage: {
						terminatingCondition: instr.terminatingCondition,
						ways: instr.ways
					}
				});

				goal.on('feedback', function(feedback) {
					var POSITION = true;
					var ORIENTATION = false;
					if (feedback.mode === POSITION) {
						if (VERBOSE) console.log('Position Mode');
						draw_pos_orien(self.ctx,3,300,400,position_color_url,position_bw_url, orient_color_url, orient_bw_url,'pos');
					} else if (feedback.mode === ORIENTATION) {
						if (VERBOSE) console.log('Orientation Mode');
						draw_pos_orien(self.ctx,3,300,400,position_color_url,position_bw_url, orient_color_url, orient_bw_url,'orien');
					}
				});

				goal.on('result', function(result) {
					if (VERBOSE) console.log('Exited cuff interaction.');
					self.ctx.clearRect(3, 300, 403, 1100)
					self.start(self.getNextAddress(instructionAddr));
				});

				goal.send();

				break;

			case 'drawPosOrien':
				var orient_bw_url = DIR + 'images/orientation_bw.png';
				var orient_color_url = DIR + 'images/orientation_color.png';
				var position_bw_url = DIR + 'images/position_bw.png';
				var position_color_url = DIR + 'images/position_color.png';

				draw_pos_orien(self.ctx,3,300,400,position_color_url,position_bw_url, orient_color_url, orient_bw_url)

				this.start(self.getNextAddress(instructionAddr));

				break;

			case 'encode':
				self.displayOff();
				canvas_container.style.display = 'initial';
				this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
				var encoder_moving_part_url = DIR + 'images/moving_part.png';
				var encoder_still_part_url = DIR + 'images/still_part.png';
				var encoder_motor_url = DIR + 'images/motor_body.png';
				var encoder_motor_circle_url = DIR + 'images/motor_circle.png';
				encoder = draw_encoder(this.ctx, encoder_moving_part_url, encoder_still_part_url, encoder_motor_url, encoder_motor_circle_url);

				this.start(self.getNextAddress(instructionAddr));

				break;

			case 'stopEncode':
			// this is a temporary command to separate clearInterval from drawing command.
			// It needs to be deleted eventually.
				clearInterval(encoder);
				this.start(self.getNextAddress(instructionAddr));
				break;

			case 'gripper':
				checkInstruction(instr, ['grip'], instructionAddr);

				var goal_Gripper = new ROSLIB.Goal({
					actionClient: this.GripperAct,
					goalMessage:{grip: instr.grip}
				});
				goal_Gripper.on('result', function(result){
					self.start(self.getNextAddress(instructionAddr));
				});
				goal_Gripper.send();

				break;

			case 'goToJointAngles':
				checkInstruction(instr, ['joint_angles'], instructionAddr);

				// Format goal.
				var goal;
				if (instr.hasOwnProperty('speed_ratio')) {
					goal = this.getGoToGoal(instr.joint_angles, instr.speed_ratio);
				} else if (instr.hasOwnProperty('wait')){
					goal = this.getGoToGoal(instr.joint_angles, 0, instr.wait);
				} else {
					goal = this.getGoToGoal(instr.joint_angles);
				}

				// When the motion is completed, move to the next instruction.
				goal.on('result', function(result) {
					self.start(self.getNextAddress(instructionAddr));
				});
				
				// Tell the robot to move.
				goal.send();

				break;

			case 'goToCartesianPose':
				checkInstruction(instr, ['position','orientation','relative_pose','joint_angles','endpoint_pose'], instructionAddr);

				var goal_GoToCartesianPose = new ROSLIB.Goal({
					actionClient: this.GoToCartesianPoseAct,
					goalMessage:{
						position: this.hashTokeyVal(instr.position),
						orientation: this.hashTokeyVal(instr.orientation),
						relative_pose: this.hashTokeyVal(instr.relative_pose),
						joint_angles: this.hashTokeyVal(instr.joint_angles),
						endpoint_pose: this.hashTokeyVal(instr.endpoint_pose)
					}
				});
				goal_GoToCartesianPose.on('result', function(result){
					self.start(self.getNextAddress(instructionAddr));
				})
				goal_GoToCartesianPose.send();

				break;

			case 'if':
				checkInstruction(instr, ['conditional','if_true'], instructionAddr);
				console.log((this.hashTokeyVal(instr.conditional)))
				if (eval(this.hashTokeyVal(instr.conditional))) {
					instructionAddr.push(true);
					instructionAddr.push(0);
					this.start(instructionAddr);
				} else if (instr.hasOwnProperty('if_false')) {
					instructionAddr.push(false);
					instructionAddr.push(0);
					this.start(instructionAddr);
				} else {
					this.start(this.getNextAddress(instructionAddr));
				}

				break;

			case 'displayImage':

				var image_url = DIR + instr.location;

				this.displayOff();

				image.src = image_url;
				image.style.display = 'initial';

				this.start(this.getNextAddress(instructionAddr));

				break;

			case 'interaction_control':
				checkInstruction(instr, ["position_only","orientation_x","orientation_y","orientation_z","position_x","position_y","position_z", "in_end_point_frame", "PASS", "ways"], instructionAddr);

				var orient_bw_url = DIR + 'images/orientation_bw.png';
				var orient_color_url = DIR + 'images/orientation_color.png';
				var position_bw_url = DIR + 'images/position_bw.png';
				var position_color_url = DIR + 'images/position_color.png';

				var goal = new ROSLIB.Goal({
					actionClient: self.InteractionControlAct,
					goalMessage: {
						position_only: instr.position_only,
						position_x: instr.position_x,
						position_y: instr.position_y,
						position_z: instr.position_z,
						orientation_x: instr.orientation_x,
						orientation_y: instr.orientation_y,
						orientation_z: instr.orientation_z,
						in_end_point_frame: instr.in_end_point_frame,
						PASS: instr.PASS,
						ways: instr.ways
					}
				});

				if (instr.hasOwnProperty('wait') && instr.wait) {
					goal.on('result', function(result) {
						self.ctx.clearRect(3, 300, 403, 1100);
						self.start(self.getNextAddress(instructionAddr));
					});
					goal.send();
				} else {
					goal.send();
					this.start(self.getNextAddress(instructionAddr));
				}

				break;

			case 'wait':
				this.wait(instr, instructionAddr).then((msg) => {
					this.start(self.getNextAddress(instructionAddr));
				});
				
				/*
				var goal = new ROSLIB.Goal({
					actionClient: this.WaitAct,
					goalMessage:{ 
						what: instr.what,
						timeout: instr.timeout 
					}
				});

				goal.on('result', result => {
					if (instr.hasOwnProperty('store_answer_in')) {
							self.dictionary[instr.store_answer_in] = result.success;
						}
					self.start(self.getNextAddress(instructionAddr));
				});
				goal.send();
				*/
				break;

			case 'joint_impedance':
				checkInstruction(instr, ["terminatingCondition","tics"], instructionAddr);

				var goal = new ROSLIB.Goal({
					actionClient: this.JointImpedanceAct,
					goalMessage: {
						terminatingCondition: this.hashTokeyVal(instr.terminatingCondition),
						tics: instr.tics
					}
				});
				goal.send();

				this.start(self.getNextAddress(instructionAddr));

				break;

			case 'joint_move':
				checkInstruction(instr, ["joints","terminatingCondition","resetPOS","min_thresh","bias","listen"], instructionAddr);

				var goal;
				if (instr.hasOwnProperty('tol')) {
					goal = this.getJointMoveGoal(instr.joints, instr.terminatingCondition, instr.resetPOS, instr.min_thresh, instr.bias, instr.tol);
				} else {
					goal = this.getJointMoveGoal(instr.joints, instr.terminatingCondition, instr.resetPOS, instr.min_thresh, instr.bias);
				}

				if (instr.listen == false){
					if(instr.hasOwnProperty('wait')){
						goal.on('result', function(result) {
							self.start(self.getNextAddress(instructionAddr));
						});
						goal.send();
					} else{
						goal.send();
						self.start(self.getNextAddress(instructionAddr));
					}
				} else{
					if(instr.hasOwnProperty('wait')){
						goal.on('result', function(result) {
							self.start(self.getNextAddress(instructionAddr));
						});
						goal.send();
					} else{
						goal.on('result', function(result) {
							self.start(self.getNextAddress(instructionAddr));
						});
						goal.send();
					}
				}

				break;

			case 'shadow_projection':

				console.log("Entered projection mode.");

				checkInstruction(instr, ["joints","terminatingCondition","resetPOS","min_thresh","bias"], instructionAddr);

				var goal = this.getJointMoveGoal(instr.joints, instr.terminatingCondition, instr.resetPOS, instr.min_thresh, instr.bias);

				

				

				goal.on('result', function(result) {
					self.endpoint.unsubscribe();
					self.endpoint.removeAllListeners();
					self.displayOff();
					self.start(self.getNextAddress(instructionAddr));
				});
				goal.send();

				break;

			case 'log':
				checkInstruction(instr, ['message'], instructionAddr);

				console.log(instr.message);

				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'multiple_choice':
				var currentGraphicMode = this.graphic_mode;
				this.set_graphic_mode({"mode":"multiple_choice"}, instructionAddr);

				var goal = new ROSLIB.Goal({
					actionClient: this.MultipleChoiceAct,
					goalMessage: {on: true}
				});

				goal.on('result', function(result) {
					if (VERBOSE) console.log('Button pressed:' + result.answer);
					self.dictionary[instr.store_answer_in] = result.answer;
					self.set_graphic_mode(currentGraphicMode, instructionAddr);
					self.start(self.getNextAddress(instructionAddr));
				});

				goal.send();

				break;

			case 'play':
				checkInstruction(instr, ['audio_index','delay'], instructionAddr);
				if (instr.hasOwnProperty('//')) {
					if (!instr['//'].startsWith('#') && instr['//'] !== this.thisSection._textArray[instr.audio_index]) {
						console.warn(`Play instruction at ${instructionAddr} has a mismatch between the "//" tag and the actual text displayed.`);
					}
				} else {
					if (VERBOSE) console.warn('It is recommended that all "play" instructions have an accompanying "//" tag with the played text for readability.');
				}

				var audio_index = instr.audio_index;
				if (typeof audio_index === 'string') {
					audio_index = eval(this.hashTokeyVal(audio_index));
				}

				this.play(this.thisSection._audiofiles_mp3[audio_index], this.thisSection._audio_duration[audio_index], this.thisSection._textArray[audio_index]);
				if (instr.delay) {
					await sleep(this.thisSection._audio_duration[audio_index]);
				}

				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'playVideo':
				this.displayOff();
				animator.style.display = 'initial';
				animator.play();

				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'pos_orient':
				var orient_bw_url = DIR + 'images/orientation_bw.png';
				var orient_color_url = DIR + 'images/orientation_color.png';
				var position_bw_url = DIR + 'images/position_bw.png';
				var position_color_url = DIR + 'images/position_color.png';

				draw_pos_orien(self.ctx,3,300,400,position_color_url,position_bw_url, orient_color_url, orient_bw_url)

				this.start(this.getNextAddress(instructionAddr));

				break;

			case 'programming_choices':
				this.ctx.font = "60px Arial";
				this.ctx.fillText("Close Gripper", 10, 50);


				this.start(this.getNextAddress(instructionAddr));

				break;

			case 'pressed_button':
				this.pressed.subscribe(async function(message) {
					if (VERBOSE) console.log('Pressed: ' + message.data);
					if (message.data == true) {
						self.pressed.unsubscribe();
						self.pressed.removeAllListeners();
						self.start(self.getNextAddress(instructionAddr));
					}
				});

				break;

			case 'proceed':
				checkInstruction(instr, ['to_section'], instructionAddr);

				this.start([instr.to_section, 0]);

				break;

			case 'projection':
				this.displayOff();
				canvas_container.style.display = 'initial';

				this.position.subscribe(async function(message) {
					if (VERBOSE) console.log(message.j1);
					draw_goal(self.ctx, 100, message.j1*400+100)
				});

				this.pressed.subscribe(async function(message) {
					if (VERBOSE) console.log('Pressed: ' + message.data);
					if (message.data == true) {
						self.position.unsubscribe();
						self.position.removeAllListeners();
						self.pressed.unsubscribe();
						self.pressed.removeAllListeners();
						self.displayOff();
						self.start(self.getNextAddress(instructionAddr));
					}
					
				});

				break;

			case "refresh":
				m.displayOff(true);

				image.style.display = 'initial';

				this.start(this.getNextAddress(instructionAddr));

				break;

			case 'reset':
				this.displayOff();

				this.ctx.clearRect(0,0,100*this.cw,100*this.ch);

				canvas_container.style.display = 'initial';

				this.start(this.getNextAddress(instructionAddr));

				break;

			case 'scrollWheelInput':
				var currentGraphicMode = this.graphic_mode;
				this.set_graphic_mode({'mode': 'scroll wheel'}, instructionAddr);

				var goal = this.getGoToGoal('joint_buttons')
				goal.on('result', function(result) {
					self.ScrollWheelSubscriptionSrv.callService(new ROSLIB.ServiceRequest({subscribe: true}), result => {
						self.wheel_delta_topic.subscribe(async function(message) {
							if (VERBOSE) console.log('Rx: ' + message.data);
								wheel_val+= message.data;
						});

						self.scroll_wheel_button_receiver.subscribe(async function(message) {
							if (VERBOSE) console.log('Scroll wheel button pressed.');
							if (instr.hasOwnProperty('store_answer_in')) {
								self.dictionary[instr.store_answer_in] = wheel_val;
							}
							self.wheel_delta_topic.unsubscribe();
							self.wheel_delta_topic.removeAllListeners();
							self.scroll_wheel_button_receiver.unsubscribe();
							self.scroll_wheel_button_receiver.removeAllListeners();
							cancel_odometer();
							self.set_graphic_mode(currentGraphicMode, instructionAddr);
							self.start(self.getNextAddress(instructionAddr));
						});
					});
				});
				goal.send();

				break;

			case 'set':
				checkInstruction(instr, ['key','val'], instructionAddr);

				if (VERBOSE) console.log(`Setting #${instr.key} to ${instr.val}`);
				if (typeof instr.val === 'string') {
					this.dictionary[instr.key] = eval(this.hashTokeyVal(instr.val));
				} else {
					this.dictionary[instr.key] = instr.val;
				}
				
				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'set_graphic_mode':
				this.set_graphic_mode(instr, instructionAddr);
				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'set_robot_mode':
				this.set_robot_mode(instr, instructionAddr);
				this.start(this.getNextAddress(instructionAddr));
				break;


			case 'show_camera':
				// this.displayOff();
				// var cv_image = new Image();

				// cv_image.onload = function() {
				// 	console.log('Displaying camera')
		  //           self.ctx.drawImage(cv_image, 20, 70)

		  //           self.start(self.getNextAddress(instructionAddr));
				// };
				// cv_image.src = DIR + 'images/cv_image.png';

				var cv_image_url = DIR + 'images/cv_image.png';

				image.src = cv_image_url;
				
				this.start(this.getNextAddress(instructionAddr));

				break;

			case 'update':

				this.getEndpoint()

				this.start(this.getNextAddress(instructionAddr));

				break; 

			case 'while':
				checkInstruction(instr, ['conditional'], instructionAddr);
		
				if (eval(this.hashTokeyVal(instr.conditional))) {
					instructionAddr.push(0);
					this.start(instructionAddr);
				} else {
					this.start(this.getNextAddress(instructionAddr));
				}
			
				break;
				
			default:
				throw `Did not recognize instruction of type ${instr.type}`;
		}
	}
}

/**
 * Replaces all references with a '#' with the value they reference from the dictionary.
 *
 * For each instance of '#' in a given string, this function replaces the '#' and the following variable reference with the value it references
 * from the dictionary. If the input is not a string, it is returned.
 *
 * Example:
 *  >> module.dictionary.a = 314;
 *  >> console.log(module.hashTokeyVal('I have #a apples.'));
 *     I have 314 apples.
 *
 * @param {string}	str	A string to process.
 * @return {object}	The processed string with references replaced.
 */
Module.prototype.hashTokeyVal = function(str) {
	switch (typeof str) {
		case 'string':
			return str.replace(/#[a-z_0-9]+/gi, key => this.dictionary[key.substring(1)].toString());
		default:
			return str;
	}
}

/**
 * Increments the current instruction address.
 *
 * If the current instructions list is not yet completed, simply increments the last element.
 * Otherwise, backs up one parent instruction.
 * If the module is completed, returns an Array with one element: the ID of the last section.
 *
 * @param {Array}	instructionAddr 	See play().
 * @return {Array}	Address of next instruction to play.
 */
Module.prototype.getNextAddress = function(instructionAddr) {
	if (instructionAddr[instructionAddr.length-1] < this.instructionSets[this.instructionSets.length-1].length-1) {
		instructionAddr[instructionAddr.length-1]++;
		return instructionAddr;
	} else {
		this.instructionSets.pop();
		instructionAddr.pop();
		if (typeof instructionAddr[instructionAddr.length-1] === 'boolean') {
			instructionAddr.pop();
			return this.getNextAddress(instructionAddr);
		} else {
			return instructionAddr;
		}
	}
}


/**
 * Checks an instruction to make sure it has the required properties.
 *
 * If the given instruction does not have the required properties, throws and error to alert the user.
 *
 * @param {object}	instruction 	The instruction object from the JSON file to be checked.
 * @param {Array}	properties		A list of strings of the required properties.
 * @param {Array}	addr 			Address of given instruction so it can alert the user if there is a problem.
 */
function checkInstruction(instruction, properties, addr) {
	for (let p=0; p<properties.length; p++) {
		if (!instruction.hasOwnProperty(properties[p])) {
			throw `Instruction ${instruction.type} at ${addr} requires property ${properties[p]}.`
		}
	}
}