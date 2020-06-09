/**
 * Draw a spring.
 *
 * Draws a spring object on a canvas. 
 *
 * @param {object}	canvas	The canvas on which to draw.
 * @param {number}	ox		The x position of the spring's start point in pixels.
 * @param {number}	oy		The y position of the spring's start point in pixels.
 * @param {number}	L		The length of the spring in pixels.
 * @param {number}	A		The width of the spring/amplitude of the sine wave drawing.
 */
function draw_spring(canvas,ox,oy,L,A) {
	var ctx = canvas.getContext('2d');
    ctx.clearRect(0,0,canvas.width,canvas.height);                  // Erase canvas

    ctx.beginPath();                                                // Draw reference line
    ctx.moveTo(100,10);
    ctx.lineTo(100,300);
    ctx.strokeStyle='black';
    ctx.stroke();

    let P=L/3;                                                      // Draw spring
    ctx.beginPath();
    ctx.moveTo(ox,oy);
    ctx.lineTo(ox+P/2,oy);
    for (var x=0;x<2*P;x++) {
        ctx.lineTo(ox+P/2+x,oy+A*Math.sin(2*Math.PI*x/P));
    }
    x=2*P;
    ctx.lineTo(ox+P/2+x,oy+A*Math.sin(2*Math.PI*x/P));
    ctx.lineTo(ox+3*P,oy);
    ctx.stroke();
    ctx.closePath();

    draw_ball(ctx,ox+3*P,oy,30)
}