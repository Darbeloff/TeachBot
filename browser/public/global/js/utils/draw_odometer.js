/**
 * Draws and animates the on-screen odometer.
 *
 * Displays the odometer on the screen and animates it when wheel_val is changed.
 *
 * @param {object}  ctx_in        The context in which to animate.
 * @param {boolean} odometer_url  The image that needs to be uploaded.
 * @param {number}  wheel_val     The value that should be displayed.
 */
function draw_odometer(ctx_in, odometer_url, wheel_val) {
    let cw = ctx_in.canvas.width;
    var imgb = new Image();

    imgb.onload = function(){
        //the original image is 1920x889
        //percentages for the blank slots on the png
        ctx_in.clearRect(0,0,ctx_in.canvas.width,ctx_in.canvas.height); 
        var choice_x_pos = 485; //these are the optimal x and y positions of the text
        var choice_y_pos = 285;

        var img_scale = .9
        var img_height = ctx_in.canvas.height*img_scale;
        font_size = img_height / 2.5;
        var img_width =  ctx_in.canvas.width*img_scale;
        var scale = img_height/425
        ctx_in.drawImage(imgb, ctx_in.canvas.width/2 - img_width/2,ctx_in.canvas.height/2 - img_height/2, width = img_width, height = img_height);
        ctx_in.font = font_size +  "px Raleway";
        ctx_in.textAlign = "center";
        ctx_in.fillStyle = "#373737";
        ctx_in.strokeStyle = '#333333';
        ctx_in.lineWidth = 5;
        ctx_in.fillText(wheel_val, ctx_in.canvas.width/2  - img_width/2 + (choice_x_pos)*img_height/425, ctx_in.canvas.height/2-img_height/2 + (choice_y_pos) * img_height/425);
        ctx_in.beginPath(); 
        ctx_in.stroke();

        var headlen = 10;
        var fromx = 325;
        var fromy = 500;
        var tox = 275;
        var toy = 775;
        var angle = Math.atan2(toy-fromy,tox-fromx);

        //starting path of the arrow from the start square to the end square and drawing the stroke
        ctx_in.beginPath();
        ctx_in.moveTo(fromx, fromy);
        ctx_in.lineTo(tox, toy);
        // ctx_in.strokeStyle = "#cc0000";
        ctx_in.lineWidth = 17;
        ctx_in.stroke();

        //starting a new path from the head of the arrow to one of the sides of the point
        ctx_in.beginPath();
        ctx_in.moveTo(tox, toy);
        ctx_in.lineTo(tox-headlen*Math.cos(angle-Math.PI/7),toy-headlen*Math.sin(angle-Math.PI/7));

        //path from the side point of the arrow, to the other side point
        ctx_in.lineTo(tox-headlen*Math.cos(angle+Math.PI/7),toy-headlen*Math.sin(angle+Math.PI/7));

        //path from the side point back to the tip of the arrow, and then again to the opposite side point
        ctx_in.lineTo(tox, toy);
        ctx_in.lineTo(tox-headlen*Math.cos(angle-Math.PI/7),toy-headlen*Math.sin(angle-Math.PI/7));

        //draws the paths created above
        // ctx_in.strokeStyle = "#cc0000";
        ctx_in.lineWidth = 22;
        ctx_in.stroke();
        // ctx_in.fillStyle = "#cc0000";
        ctx_in.fill();

    }
    imgb.src = odometer_url;
}

var odometer_url = 'https://localhost:8000/images/new_scroll.JPG';