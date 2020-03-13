const ROBOT = 'sawyer';
const DIR = 'https://localhost:8000/';
var current_angles = [];
var current_angles_raw = [];
var current_position = [];
var current_orientation = [];

const common_positions = {
	'home': [0.0, -0.78, 0.0, 1.57, 0, -0.79, 0.2],
	'upright': [0, -1.57, 0, 0, 0, 0, 0.2],
	'scara_initial': [0.78, -0.06, 1.57, -1.57, 2.99, -0.78, 1.80],
	'2dPos': [0.90060,-0.05870,1.56901,-1.56918,2.97392,-1.57146,1.79917],
	'm3_test1':[0.51250,-0.15236,1.53013,-1.97725,2.93612,-1.53143,1.64995]
}

var ros = new ROSLIB.Ros({ url: 'wss://localhost:9090' });
ros.on('connection', function() { console.log('Connected to websocket server.'); });
ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); window.alert('Error connecting to websocket server'); });
ros.on('close', function() { console.log('Connection to websocket server closed.'); });

// Subscribing topics
var position = new ROSLIB.Topic({
	ros: ros,
	name: '/teachbot/position',
	messageType: ROBOT + '/JointInfo'
});

var endpoint = new ROSLIB.Topic({
	ros: ros,
	name: '/teachbot/EndpointInfo',
	messageType: ROBOT + '/EndpointInfo'
});


// Action clients
var AllowCuffInteractionAct = new ROSLIB.ActionClient({
	ros: ros,
	serverName: '/teachbot/AllowCuffInteraction',
	actionName: ROBOT + '/AllowCuffInteractionAction'
});

var GoToJointAnglesAct = new ROSLIB.ActionClient({
	ros: ros,
	serverName: '/teachbot/GoToJointAngles',
	actionName: ROBOT + '/GoToJointAnglesAction'
});

var JointMoveAct = new ROSLIB.ActionClient({
	ros: ros,
	serverName: '/teachbot/JointMove',
	actionName: ROBOT + '/JointMoveAction'
});

position.subscribe(async function(message) {
	document.getElementById("j0").innerHTML = message.j0.toFixed(2);
	document.getElementById("j1").innerHTML = message.j1.toFixed(2);
	document.getElementById("j2").innerHTML = message.j2.toFixed(2);
	document.getElementById("j3").innerHTML = message.j3.toFixed(2);
	document.getElementById("j4").innerHTML = message.j4.toFixed(2);
	document.getElementById("j5").innerHTML = message.j5.toFixed(2);
	document.getElementById("j6").innerHTML = message.j6.toFixed(2);
	current_angles = [message.j0.toFixed(2), message.j1.toFixed(2), message.j2.toFixed(2), message.j3.toFixed(2), message.j4.toFixed(2), message.j5.toFixed(2), message.j6.toFixed(2)];
	current_angles_raw = [message.j0.toFixed(5), message.j1.toFixed(5), message.j2.toFixed(5), message.j3.toFixed(5), message.j4.toFixed(5), message.j5.toFixed(5), message.j6.toFixed(5)];
});

endpoint.subscribe(async function(message) {
	document.getElementById("pos_x").innerHTML = message.position.x.toFixed(2);
	document.getElementById("pos_y").innerHTML = message.position.y.toFixed(2);
	document.getElementById("pos_z").innerHTML = message.position.z.toFixed(2);
	document.getElementById("orien_x").innerHTML = message.orientation.x.toFixed(2);
	document.getElementById("orien_y").innerHTML = message.orientation.y.toFixed(2);
	document.getElementById("orien_z").innerHTML = message.orientation.z.toFixed(2);
	document.getElementById("orien_w").innerHTML = message.orientation.w.toFixed(2);
	current_position = [message.position.x.toFixed(5), message.position.y.toFixed(5), message.position.z.toFixed(5)];
	current_orientation = [message.orientation.x.toFixed(5),message.orientation.y.toFixed(5),message.orientation.z.toFixed(5),message.orientation.w.toFixed(5)];
	if (document.title=='Cuff Interaction') {
		xyzw = current_orientation;
		degree = true;
		if (Math.abs(xyzw[0]*xyzw[1]+xyzw[2]*xyzw[3]-0.5)<0.00001) {
			document.getElementById("heading").innerHTML = (2*Math.atan2(xyzw[0], xyzw[3]) * (degree?180/Math.PI:1)).toFixed(2);
			document.getElementById("attitude").innerHTML = (Math.asin(2*xyzw[0]*xyzw[1]+2*xyzw[2]*xyzw[3]) * (degree?180/Math.PI:1)).toFixed(2);
			document.getElementById("bank").innerHTML = 0.0 * (degree?180/Math.PI:1);
		} else if (Math.abs(xyzw[0]*xyzw[1]+xyzw[2]*xyzw[3]+0.5)<0.00001) {
			document.getElementById("heading").innerHTML = (-2*Math.atan2(xyzw[0], xyzw[3]) * (degree?180/Math.PI:1)).toFixed(2);
			document.getElementById("attitude").innerHTML = (Math.asin(2*xyzw[0]*xyzw[1]+2*xyzw[2]*xyzw[3]) * (degree?180/Math.PI:1)).toFixed(2);
			document.getElementById("bank").innerHTML = 0.0 * (degree?180/Math.PI:1);
		} else {
			document.getElementById("heading").innerHTML = (Math.atan2(2*xyzw[1]*xyzw[3]-2*xyzw[0]*xyzw[2], 1-2*Math.pow(xyzw[1],2)-2*Math.pow(xyzw[2],2)) * (degree?180/Math.PI:1)).toFixed(2);
			document.getElementById("attitude").innerHTML = (Math.asin(2*xyzw[0]*xyzw[1]+2*xyzw[2]*xyzw[3]).toFixed(2) * (degree?180/Math.PI:1)).toFixed(2);
			document.getElementById("bank").innerHTML = (Math.atan2(2*xyzw[0]*xyzw[3]-2*xyzw[1]*xyzw[2], 1-2*Math.pow(xyzw[0],2)-2*Math.pow(xyzw[2],2)) * (degree?180/Math.PI:1)).toFixed(2);
		}
	}
});

function clearInputFields() {
	document.getElementById("g_j0").value = "";
	document.getElementById("g_j1").value = "";
	document.getElementById("g_j2").value = "";
	document.getElementById("g_j3").value = "";
	document.getElementById("g_j4").value = "";
	document.getElementById("g_j5").value = "";
	document.getElementById("g_j6").value = "";
}

function goToPosition(pos) {
	var goal = new ROSLIB.Goal({
		actionClient: GoToJointAnglesAct,
		goalMessage: {
			j0pos: common_positions[pos][0],
			j1pos: common_positions[pos][1],
			j2pos: common_positions[pos][2],
			j3pos: common_positions[pos][3],
			j4pos: common_positions[pos][4],
			j5pos: common_positions[pos][5],
			j6pos: common_positions[pos][6],
			speed_ratio: 0.5
		}
	});
	goal.on('result', function(result) {
		clearInputFields();
	});
	goal.send();
}

function sendJointAngles(g0, g1, g2, g3, g4, g5, g6) {
	var desired_angles = [g0, g1, g2, g3, g4, g5, g6];
	for(var i=0;i<desired_angles.length;i++) {
		if (desired_angles[i]=="") {
			desired_angles[i] = parseFloat(current_angles[i]);
		} else {
			desired_angles[i] = parseFloat(desired_angles[i]);
		}
	};
	var newPositionGoal = new ROSLIB.Goal({
		actionClient: GoToJointAnglesAct,
		goalMessage: {
			j0pos: desired_angles[0],
			j1pos: desired_angles[1],
			j2pos: desired_angles[2],
			j3pos: desired_angles[3],
			j4pos: desired_angles[4],
			j5pos: desired_angles[5],
			j6pos: desired_angles[6],
			speed_ratio: 0.2
		}
	});
	newPositionGoal.on('result', function(result) {
		clearInputFields();
	});
	newPositionGoal.send();
};

function enableCuffInteraction() {
	var goal = new ROSLIB.Goal({
		actionClient: AllowCuffInteractionAct,
		goalMessage:{
			allow:true
		}
	});
	goal.on('result', function(result) {
		console.log(result.status);
	});
	goal.send();
}

function disableCuffInteraction() {
	var goal = new ROSLIB.Goal({
		actionClient: AllowCuffInteractionAct,
		goalMessage:{
			allow:false
		}
	});
	goal.on('result', function(result) {
		position.unsubscribe();
		endpoint.unsubscribe();
	});
	goal.send();
}

function anglesToClipboard() {
	var dummy = document.createElement("textarea");
    document.body.appendChild(dummy);
    dummy.value = "["+current_angles_raw+"]";
    dummy.select();
    document.execCommand("copy");
    document.body.removeChild(dummy);
    document.getElementById("copy_complete").innerHTML = "Copied to clipboard!";
}

function outFunc() {
	document.getElementById("copy_complete").innerHTML = "";
}