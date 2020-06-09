/**
 * Draw an arc.
 *
 * Draws an arc with three points. 
 *
 * @param {object}   ctx_in  The context of the canvas on which to draw.
 * @param {number}   x1      The x position of the first guide point in pixels.
 * @param {number}   y1      The y position of the first guide point in pixels.
 * @param {number}   x2      The x position of the second guide point in pixels.
 * @param {number}   y2      The y position of the second guide point in pixels.
 * @param {number}   x3      The x position of the third guide point in pixels.
 * @param {number}   y3      The y position of the third guide point in pixels.
 * @param {boolean}  ccw     Draw counterclockwise, as opposed to clockwise.
 */
function arc3pt(ctx_in, x1, y1, x2, y2, x3, y3, ccw) {
    var yc = ((x3*x3-x1*x1+y3*y3-y1*y1)*(x2-x1)-(x3-x1)*(x2*x2-x1*x1+y2*y2-y1*y1))/2/((y3-y1)*(x2-x1)-(x3-x1)*(y2-y1));
    var xc = (x2*x2-x1*x1+y2*y2-y1*y1-2*(y2-y1)*yc)/2/(x2-x1);
    var r = Math.sqrt(Math.pow(x1-xc,2)+Math.pow(y1-yc,2));
    // console.log('C = ( ' + xc + ' , ' + yc + ' );\nr = ' + r)
    ctx_in.beginPath();
    ctx_in.strokeStyle = '#000000';
    ctx_in.arc(xc,yc,r,Math.atan2(y1-yc,x1-xc),Math.atan2(y3-yc,x3-xc),ccw);
    ctx_in.stroke();
    ctx_in.closePath();
}