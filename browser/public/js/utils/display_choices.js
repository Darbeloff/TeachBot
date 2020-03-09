/**
 * Display multiple choice answers.
 *
 * Draws multiple choice image and overlays choices.
 *
 * @param {object}      ctx_in              The context of the canvas on which to draw.
 * @param {String[]}    choices_arr         Array of five possible answers to the question.
 * @param {String[]}    multi_choice_url    Location of the background image showing the gripper cuff.
 */

function display_choices(ctx_in, choices_arr, multi_choice_url, code=false, program_arr = [], x= 0 ,y= 0) {
    let cw = ctx_in.canvas.width;
    console.log("displaying choices")
    var imgb = new Image();

    imgb.onload = function(){
        //the original image is 1920x889
        //percentages for the blank slots on the png
        ctx_in.clearRect(0,0,ctx_in.canvas.width,ctx_in.canvas.height); 

        var choice_x_pos = [250,260,450,415,230,460]; //these are the optimal x and y positions of the text
        var choice_y_pos = [205,358,358,205, 80,80];
        var pos_x_line_end = [320,295,485,460,290,480]; //these are the optimal x and y positions of the text
        var pos_y_line_end = [215,338,338,215,90,90];
        var pos_x_line = [270,350,435,520,350,435];//these are the optimal x and y positions on the original image
        var pos_y_line = [275,275,275,275,115,115];
        var img_scale = .9
        var img_height = ctx_in.canvas.height*img_scale;
        font_size = img_height / 20;
        console.log("font" + font_size);
        console.log(img_scale);
        var img_width =  780/425*img_height;
        var scale = img_height/425
        ctx_in.drawImage(imgb, ctx_in.canvas.width/2 - img_width/2,ctx_in.canvas.height/2 - img_height/2, width = img_width, height = img_height);
        ctx_in.font = font_size +  "px Raleway";
        ctx_in.textAlign = "left";
        ctx_in.fillStyle = "#373737";
        ctx_in.strokeStyle = '#333333';
        for (let c=0; c<choices_arr.length; c++) {
            console.log("hello")
            ctx_in.lineWidth = 2;
            ctx_in.fillText(choices_arr[c], ctx_in.canvas.width/2  - img_width/2 + (choice_x_pos[c] )*img_height/425, ctx_in.canvas.height/2-img_height/2 + (choice_y_pos[c] ) * img_height/425);
            ctx_in.beginPath(); 
            ctx_in.moveTo(ctx_in.canvas.width/2  - img_width/2 + (pos_x_line[c] )*img_height/425, ctx_in.canvas.height/2-img_height/2 + (pos_y_line[c] ) * img_height/425)
            ctx_in.lineTo(ctx_in.canvas.width/2  - img_width/2 + (pos_x_line_end[c] )*img_height/425, ctx_in.canvas.height/2-img_height/2 + (pos_y_line_end[c] ) * img_height/425)
            ctx_in.stroke();
/*            ctx_in.restore();*/
        }

        var headlen = 10;
        var fromx = 575;
        var fromy = 500;
        var tox = 375;
        var toy = 800;
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

        if (code){
            ctx_in.clearRect(0,0,300,ctx_in.canvas.height); 
            ctx_in.font = "40px Raleway";
            ctx_in.textAlign = "left";
            ctx_in.fillStyle = "#373737";
            ctx_in.strokeStyle = '#333333';
            ctx_in.lineWidth = 7;
            var spacing = 75;

            if (program_arr.length > 10){
                spacing = spacing - 5*(program_arr.length-10);
                var temp = 40-1*(program_arr.length-10)
                ctx_in.font = temp.toString() + 'px Raleway';
            }
            for (let c=0; c<program_arr.length; c++){
                ctx_in.fillText(program_arr[c], x, y+spacing*c); 
                ctx_in.beginPath(); 
                ctx_in.stroke();
            }
        }
        ctx_in.lineWidth = 2;
    }
    imgb.src = multi_choice_url;

}