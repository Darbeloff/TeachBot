/**
 * Draw a rectangle.
 *
 * Draws an unfilled rectangle object on a canvas. 
 *
 * @param {object}	ctx_in			The context of the canvas on which to draw.
 * @param {number}	x				The x position of the rectangle's top left corner in pixels.
 * @param {number}	y				The y position of the rectangle's top left corner in pixels.
 * @param {number}	w 				The width of the rectangle in pixels.
 * @param {number}	h 				The height of the rectangle in pixels.
 * @param {number}	rotate=0		Angles in degrees to rotate, positive in cw and negative in ccw direction.
 * @param {string}  label=null		The label to display under the rectangle.
 */
function draw_rectangle(ctx_in,x,y,w,h,rotate=0,label=null) {
	ctx_in.save();
    ctx_in.beginPath();
    ctx_in.translate(x+w/2, y+h/2);
    ctx_in.rotate(rotate*Math.PI/180);
    ctx_in.rect(-w/2,-h/2,w,h);
    ctx_in.stroke();
    ctx_in.restore();

    if (label!==null) {
    	fontSize = parseInt(/([0-9]+)px/g.exec(ctx_in.font)[1]);
    	ctx_in.textAlign = "center";
    	// Label location currently not working properly.
    	rotate ? ctx_in.fillText(label, x+h/2, y+w+1.2*fontSize) : ctx_in.fillText(label,x+w/2, y+h+1.2*fontSize);
    }
}