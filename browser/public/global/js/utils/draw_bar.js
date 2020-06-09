/**
 * Draw a bar chart.
 *
 * Draws a bar chart object on a canvas. 
 *
 * @param {object}  ctx_in                  The context of the canvas on which to draw.
 * @param {number}  x                       The x position of the left-bottom corner of a bar in pixel.
 * @param {number}  y                       The y position of the left-bottom corner of a bar in pixel.
 * @param {number}  width                   The width of the bar in pixel.
 * @param {number}  max_height              The maximum height of the bar chart in pixel.
 * @param {number}  height_percent          The height of this bar in percentage of max_height.
 * @param {string}  fillStyle               The color of the ball's fill.
 * @param {string}  [label=null]            The label to display under the bar chart.
 * @param {string}  [label_align=center]    The alignment of the label, default at center.
 */
function draw_bar_new(ctx_in, x, y, width, max_height, height_percent, fillStyle, label=null, label_align="center") {
    
    // ctx_in.clearRect(0,0,ctx_in.canvas.width,ctx_in.canvas.height);

    if(label!==null){
        ctx_in.fillStyle = "#373737";
        ctx_in.strokeStyle = '#333333';
        ctx_in.beginPath(); 
        ctx_in.font = "30px Raleway";
        ctx_in.lineWidth = 2;
        var fontSize = 30;
        // fontSize = parseInt(/([0-9]+)px/g.exec(ctx_in.font)[1]);
        ctx_in.textAlign = "center"
        ctx_in.fillText(label, x+width/2, y+1.2*fontSize);
        ctx_in.stroke();
    }

    ctx_in.fillStyle = fillStyle;
    ctx_in.fillRect(x, y, width, -max_height*height_percent);
    // Draw a base line at the bottom of a bar.
    ctx_in.beginPath();
    ctx_in.moveTo(x-0.05*width,y);
    ctx_in.lineTo(x+1.05*width,y);
    ctx_in.strokeStyle = '#FFFFFF';
    ctx_in.lineWidth = 5;
    ctx_in.stroke();

 }
