/**
 * Draws encoder graphic on a canvas.
 *
 * @param {object}  ctx          The context of the canvas on which to draw.
 * @param {object}  encoder_moving_part_url         The url of image for moving part of robot arm.
 * @param {object}  encoder_moving_part_url         The url of image for still part of robot arm.
 * @param {object}  encoder_moving_part_url         The url of image for body of motor.
 * @param {object}  encoder_moving_part_url         The url of image for rotating circle of motor.
 */
function draw_encoder(ctx, encoder_moving_part_url, encoder_still_part_url, encoder_motor_url, encoder_motor_circle_url) {
	var max_turn = 60; // change in angle
	var turn = 1.5; // speed
	var direction = 1;
	var angle = 0; // start angle
	var still_part = new Image();
	var moving_part = new Image();
	var motor = new Image();
	var motor_circle = new Image();
	var canvas = ctx.canvas;

	// width and height of part of canvas containing animation
	var img_scale = 0.95;
	var w = canvas.width*img_scale;
	var h = w*.5;
	var startx = canvas.width/2 - w/2;
	var starty = canvas.height/2 - h/2;



	drawMoving = function() {
		if (direction == -1) {
			if (angle == 0) {
				direction = 1
				angle = angle+turn
			}
			else {
				angle = angle-turn
			}
		}
		else if (direction == 1){
			if (angle > max_turn) {
				direction = -1
				angle = angle-turn
			}
			else {
				angle = angle+turn
			}
		}

		// robot parts
		ctx.save();
		ctx.clearRect(0, 0, canvas.width, canvas.height);
		ctx.beginPath();
		ctx.stroke();
		ctx.beginPath();
		ctx.stroke();
		var still_part_w = w*0.2
		var still_part_h = h*0.65
		ctx.drawImage(still_part, startx+w*0.35, starty+h*0.3,still_part_w,still_part_h);
		var moving_part_w = w*0.26
		var moving_part_h = h*0.56
		var moving_part_x = startx+still_part_w*1.91;
		var moving_part_y = starty+still_part_h*0.58;
		ctx.translate(moving_part_x, moving_part_y);
		ctx.rotate(angle*Math.PI/180);
		ctx.drawImage(moving_part,-(moving_part_w*0.88),-(moving_part_h*0.07),moving_part_w, moving_part_h);
		ctx.rotate(-angle*Math.PI/180);
		ctx.translate(-moving_part_x, -moving_part_y);

		// inside view 
		// small: circle
		ctx.beginPath();
		ctx.lineWidth = 3;
		ctx.fillStyle = "#efefef";
		ctx.strokeStyle = "#1e1e1e";
		var small_circle_r = w*0.02;
		var small_circle_x = startx+w*0.385;
		var small_circle_y = starty+w*0.195;
		ctx.arc(small_circle_x, small_circle_y, small_circle_r, 0, 2 * Math.PI);
		ctx.stroke();
		ctx.fill();

		// small: motor
		var small_motor_x = w*0.013;
		var small_motor_y = w*0.007;
		var small_motor_w = small_circle_r+(w*0.008);
		var small_motor_h = small_circle_r-(w*0.006);
		var small_motor_w = small_motor_w.toFixed(3);
		var small_motor_h = small_motor_h.toFixed(3);
		var motor_scale = 3.5;
		ctx.drawImage(motor, small_circle_x-small_motor_x, small_circle_y-small_motor_y, small_motor_w, small_motor_h);

		// small: rotating circle
		var small_rot_circle_x = w*0.0065;
		var small_rot_circle_d = w*0.007;
		ctx.translate(small_circle_x-small_rot_circle_x,small_circle_y);
		ctx.rotate(angle*Math.PI/180);
		ctx.drawImage(motor_circle, -small_rot_circle_d, -small_rot_circle_d, small_rot_circle_d*2, small_rot_circle_d*2);
		ctx.rotate(-angle*Math.PI/180);
		ctx.translate(small_rot_circle_x-small_circle_x,-small_circle_y);

		// projected view
		// projected: circle
		ctx.beginPath();
		var big_circle_r = small_circle_r*motor_scale;
		var big_circle_x = startx+w*0.65;
		var big_circle_y = starty+w*0.3;
		ctx.arc(big_circle_x, big_circle_y, big_circle_r, 0, 2 * Math.PI);
		ctx.fill();
		ctx.stroke();

		// projected: motor
		var big_motor_x = small_motor_x*motor_scale;
		var big_motor_y = small_motor_y*motor_scale;
		var big_motor_w = small_motor_w*motor_scale;
		var big_motor_h = small_motor_h*motor_scale;
		ctx.drawImage(motor, big_circle_x-big_motor_x, big_circle_y-big_motor_y, big_motor_w, big_motor_h);

		// projected: rotating circle
		var big_rot_circle_x = small_rot_circle_x*motor_scale;
		var big_rot_circle_d = small_rot_circle_d*motor_scale;
		ctx.translate(big_circle_x-big_rot_circle_x,big_circle_y);
		ctx.rotate(angle*Math.PI/180);
		ctx.drawImage(motor_circle, -big_rot_circle_d, -big_rot_circle_d, big_rot_circle_d*2, big_rot_circle_d*2);
		ctx.rotate(-angle*Math.PI/180);
		ctx.translate(big_rot_circle_x-big_circle_x,-big_circle_y);

		// labels
		var font_size = w*0.012;
		ctx.font = String(font_size) + "px Raleway Bold";
        ctx.textAlign = "left";
		ctx.fillStyle = "#1e1e1e";
		ctx.fillText("motor", big_circle_x-w*0.045, big_circle_y+h*0.08);
		ctx.fillStyle = "#00c600";
		ctx.fillText("encoder", big_circle_x, big_circle_y+h*0.08);

		// dashed line connecting projected view of motor to bar chart
		ctx.beginPath();
		ctx.strokeStyle = "#00c600";
		ctx.setLineDash([5,3]);
		var green_dash_l = w*0.09;
		ctx.translate(big_circle_x-big_motor_x+big_motor_w, big_circle_y);
		ctx.moveTo(0,0);
		ctx.rotate(90*Math.PI/180);
		ctx.lineTo(0, -green_dash_l);
		ctx.stroke();
		ctx.rotate(-90*Math.PI/180);
		ctx.translate(-big_circle_x+big_motor_x-big_motor_w, -big_circle_y);

		// bar chart
		ctx.fillStyle = "#00c600";
		ctx.beginPath();
		ctx.rect(startx+w*0.795, starty+h*0.75, w*0.1, w*0.001*(angle-90));
		ctx.fill();

		// bar chart frame
		ctx.strokeStyle = "#1e1e1e";
		ctx.setLineDash([]);
		ctx.beginPath();
		ctx.lineWidth = 3;
		ctx.rect(startx+w*0.795, starty+h*0.35, w*0.1, w*0.2);
		ctx.stroke();

		// angle: moving line
		ctx.lineWidth = 3;
		ctx.setLineDash([]);
		ctx.beginPath();
		ctx.strokeStyle = "#00c600";
		ctx.translate(startx+w*0.36, starty+w*0.13);
		ctx.moveTo(0,0);
		ctx.rotate((angle+25)*Math.PI/180);
		ctx.lineTo(0, w*0.05);
		ctx.stroke();
		ctx.rotate(-(angle+25)*Math.PI/180);

		// angle: stationary line
		ctx.beginPath();
		ctx.moveTo(0,0);
		ctx.rotate(-20*Math.PI/180);
		ctx.lineTo(0, -w*0.05);
		ctx.stroke();
		ctx.rotate(20*Math.PI/180);

		// angle: fill
		ctx.beginPath();
		ctx.fillStyle = "#00c600";
		ctx.moveTo(0,0);
		ctx.arc(0,0, w*0.025, (angle+115)*Math.PI/180, 250*Math.PI/180);
		ctx.stroke();
		ctx.fill();
		ctx.translate(-(startx+w*0.36), -(starty+w*0.13));

		// line connecting images of encoder
		ctx.beginPath();
		ctx.strokeStyle = "#1e1e1e";
		ctx.moveTo(big_circle_x-big_circle_r, big_circle_y);
		ctx.lineTo(small_circle_x+small_circle_r, small_circle_y);
		ctx.stroke();
		ctx.restore();
	};


	
	moving_part.src = encoder_moving_part_url;
	still_part.src = encoder_still_part_url;
	motor.src = encoder_motor_url;
	motor_circle.src = encoder_motor_circle_url;

	var x = setInterval(drawMoving, 50);
	return x;
	
}

var encoder = 0;