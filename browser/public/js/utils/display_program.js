/**
 * Draws icons for position and orientation on a canvas, with the active icon (if any) in color.
 *
 * @param {object}  ctx_in          The context of the canvas on which to draw.
 * @param {number}  x      			The x coordinate for the text.
 * @param {number}  y      			The y coordinate for the text.
 * @param {array}   program_arr     The array of programming choices selected by the user.
 */

function display_program(ctx_in, x, y, program_arr, img_url) {
	ctx_in.clearRect(0,0,300,ctx_in.canvas.height); 
	ctx_in.font = "40px Raleway";
	ctx_in.textAlign = "left";
	ctx_in.fillStyle = "#373737";
    ctx_in.strokeStyle = '#333333';
    ctx_in.lineWidth = 7;
	for (let c=0; c<program_arr.length; c++){
		ctx_in.fillText(program_arr[c], x, y+75*c); 
		ctx_in.beginPath(); 
		ctx_in.stroke();
	}
}