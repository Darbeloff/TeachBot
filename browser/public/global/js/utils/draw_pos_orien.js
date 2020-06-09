/**
 * Draws icons for position and orientation on a canvas, with the active icon (if any) in color.
 *
 * @param {object}  ctx_in          The context of the canvas on which to draw.
 * @param {number}  x      			The x coordinate for the upper left corner of the icons.
 * @param {number}  y      			The y coordinate for the upper left corner of the icons.
 * @param {number}  width 			The width of the icons.
 * @param {string}  pos_bw       	The url of the position icon in black and white.
 * @param {string}  pos_color       The url of the position icon in color.
 * @param {string}  pos_bw       	The url of the position icon in black and white.
 * @param {string}  orien_color     The url of the orientation icon in color.
 * @param {string}  orien_bw        The url of the orientation icon in black and white.
 * @param {string}  status			status = "pos": Only the label and icon for position will be in color.
 									status = "orien": Only the label and icon for orientation will be in color.
 									For any other status, or if parameter is empty, all icons and labels will be in black and white.
 */

function draw_pos_orien(ctx_in, x, y, width, pos_color, pos_bw, orien_color, orien_bw, status = "none") {
	var pos_img = new Image();
	var orien_img = new Image();

	pos_img.onload = function(){
		ctx_in.drawImage(pos_img, x, y+width*0.1, width*0.475, width*0.475);
	}
	orien_img.onload = function(){
		ctx_in.drawImage(orien_img, x+width*0.525, y+width*0.1, width*0.475, width*0.475);
	}

	font_size = width/15;
	ctx_in.font = "bold " + font_size + "px Raleway";
	ctx_in.textAlign = "center";
	ctx_in.fillStyle = "#333333";

	if (status == "pos") {
		pos_img.src = pos_color;
		orien_img.src = orien_bw;
		ctx_in.fillText("Orientation", x+width*0.7625, y+font_size);
		ctx_in.fillStyle = "#931D1e";
		ctx_in.fillText("Position", x+width*0.25, y+font_size);
	}

	else if (status == "orien") {
		pos_img.src = pos_bw;
		orien_img.src = orien_color;
		ctx_in.fillText("Position", x+width*0.25, y+font_size);
		ctx_in.fillStyle = "#931D1e";
		ctx_in.fillText("Orientation", x+width*0.7625, y+font_size);
	}

	else {
		pos_img.src = pos_bw;
		orien_img.src = orien_bw;
		ctx_in.fillText("Position", x+width*0.25, y+font_size);
		ctx_in.fillText("Orientation", x+width*0.7625, y+font_size);
	}

	ctx_in.textAlign = "left";
}
