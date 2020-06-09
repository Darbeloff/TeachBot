/**
 * Draws a pair of bar charts with a comparison of height.
 *
 * Draws two bar charts side by side. The bar chart on the right shows a goal value. The bar chart
 * on the left shows the current value as well as the error between the current and goal values as
 * a stacked bar chart.
 *
 * @param {object}   ctx_in                     The context of the canvas on which to draw.
 * @param {number}   goal                       The height of the goal.
 * @param {number}   current                    The height of the current value.
 * @param {string}   [input_text="Current"]     The text to write underneath the "current" bar chart.
 */
function draw_goal(ctx_in, goal, current, input_text = "Current") {
    //current is the current position of the arm (usually from a rostopic)
    //goal is the static position, which the user is aiming for
    let cw = ctx_in.canvas.width;
    let ch = ctx_in.canvas.height;
    ctx_in.clearRect(0,0,cw,ch); 
    var bottom_margin = .9*ch;
    var bar_width = .3*cw;
    var error_margin = 3;
    var cluster = .07;
    //function for hatching

    //arrow
    ctx_in.textAlign= "left" 
    ctx_in.strokeStyle = "#373737";
    ctx_in.fillStyle = "#373737"
    ctx_in.lineWidth = 2;
    ctx_in.beginPath();
    ctx_in.font = cw*.03 +  "px Raleway";
    ctx_in.moveTo(cw*.055, ch*.208);
    ctx_in.fillText("Error",cw*.05, ch*.2);
    ctx_in.lineTo(cw*.105, ch*.208);
    //line to the center of the error bar 
    ctx_in.lineTo(cw*(.25+cluster)-.5*bar_width, (bottom_margin - (current*bottom_margin/180)) -(goal-current)*bottom_margin/180/2 )
    ctx_in.stroke();
    ctx_in.closePath();
    //comparison rectangle
    ctx_in.beginPath();
    ctx_in.rect(cw*(.75-cluster)-.5*bar_width, bottom_margin, bar_width, -goal*bottom_margin/180 )
    ctx_in.fillStyle = "#373737"
    ctx_in.fill();
    ctx_in.strokeStyle = "#555555";
    ctx_in.lineWidth = 3;
    ctx_in.stroke();
    ctx_in.closePath();
    ctx_in.beginPath();
    //base rectangle
    ctx_in.rect(cw*(.25+cluster)-.5*bar_width, bottom_margin, bar_width, -((current*bottom_margin/180)));
    ctx_in.fillStyle="#7C2629";
    ctx_in.fill();
    ctx_in.closePath();
    ctx_in.beginPath();
    //diff rectangle
    ctx_in.rect(cw*(.25+cluster)-.5*bar_width, (bottom_margin - (current*bottom_margin/180)), bar_width, -((goal-current)*bottom_margin/180));    
    if (current<goal){
        //yellow and red
        ctx_in.fillStyle="#9d781b";
    } else {
        //yellow
        ctx_in.fillStyle="#b79f00"
    }
    ctx_in.fill();
    //text labels
    ctx_in.fillStyle = "#373737"
    ctx_in.font = cw*.05 +  "px Raleway";
    ctx_in.textAlign= "center" 
    ctx_in.fillText(input_text, cw*(.25+cluster), ch);
    ctx_in.fillText("Desired", cw*(.75-cluster), ch); 
    //ctx_in.rect(20,20,100,150);
    //ctx_in.stroke();
    ctx_in.closePath();
}