/**
 * Display multiple choice answers.
 *
 * Draws multiple choice image and overlays choices.
 *
 * @param {object}      ctx_in              The context of the canvas on which to draw.
 * @param {String[]}    choices_arr         Array of five possible answers to the question.
 * @param {String[]}    multi_choice_url    Location of the background image showing the gripper cuff.
 */

function display_choices(ctx_in, choices_arr, multi_choice_url) {
    let cw = ctx_in.canvas.width;
    console.log("displaying choices")
    var imgb = new Image();

    imgb.onload = function(){
        //the original image is 780x425
        //percentages for the blank slots on the png
        ctx_in.clearRect(0,0,ctx_in.canvas.width,ctx_in.canvas.height); 
        var choice_x_pos = [166,559,166,530,166]; //these are the optimal x and y positions of the text
        var choice_y_pos = [105,105,241,241,340];
        var pos_x_line_end = [240,559,255,530,245]; //these are the optimal x and y positions of the text
        var pos_y_line_end = [100,95,235,236,335];
        var pos_x_line = [351,432,346,432,373];//these are the optimal x and y positions on the original image
        var pos_y_line = [139,139,286,286,329];
        var img_scale = .9
        var img_height = ctx_in.canvas.height*img_scale;
        font_size = img_height / 20;
        var img_width =  780/425*img_height;
        var scale = img_height/425
        ctx_in.drawImage(imgb, ctx_in.canvas.width/2 - img_width/2,ctx_in.canvas.height/2 - img_height/2, width = img_width, height = img_height);
        ctx_in.font = font_size +  "px Raleway";
        ctx_in.textAlign = "left";
        ctx_in.fillStyle = "#373737";
        ctx_in.lineWidth = 2;
        ctx_in.strokeStyle = '#333333';
        for (let c=0; c<choices_arr.length; c++) {
            ctx_in.fillText(choices_arr[c], ctx_in.canvas.width/2  - img_width/2 + (choice_x_pos[c] )*img_height/425, ctx_in.canvas.height/2-img_height/2 + (choice_y_pos[c] ) * img_height/425);
            ctx_in.beginPath(); 
            ctx_in.moveTo(ctx_in.canvas.width/2  - img_width/2 + (pos_x_line[c] )*img_height/425, ctx_in.canvas.height/2-img_height/2 + (pos_y_line[c] ) * img_height/425)
            ctx_in.lineTo(ctx_in.canvas.width/2  - img_width/2 + (pos_x_line_end[c] )*img_height/425, ctx_in.canvas.height/2-img_height/2 + (pos_y_line_end[c] ) * img_height/425)
            ctx_in.stroke();
/*            ctx_in.restore();*/
        }
    }
    imgb.src = multi_choice_url;
}