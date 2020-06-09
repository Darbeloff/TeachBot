/**
 * Draw a ball.
 *
 * Draws a ball object on a canvas. 
 *
 * @param {object}	ctx_in			The context of the canvas on which to draw.
 * @param {number}	cx				The x position of the ball's center point in pixels.
 * @param {number}	cy				The y position of the ball's center point in pixels.
 * @param {number}	r				The radius of the ball in pixels.
 * @param {string}	fillStyle		The color of the ball's fill.
 * @param {string}  [label=null]	The label to display next to the ball.
 */
function draw_ball(ctx_in,cx,cy,r,fillStyle,label=null) {
	if (label!==null) {
		ctx_in.fillStyle = "#373737";
        ctx_in.strokeStyle = '#333333';
        ctx_in.beginPath(); 
        ctx_in.font = "30px Raleway";
        ctx_in.lineWidth = 2;
        var fontSize = 30;
        ctx_in.textAlign = "center"
    	// fontSize = parseInt(/([0-9]+)px/g.exec(ctx_in.font)[1]);
        ctx_in.fillText(label,cx-r-0.01*ctx_in.canvas.width-fontSize*label.length,cy+fontSize/2);
        ctx_in.stroke();
    }
    
    ctx_in.fillStyle = fillStyle;
    ctx_in.beginPath();
    ctx_in.arc(cx,cy,r,0,2*Math.PI,false);
    ctx_in.fill();
    ctx_in.closePath();
}