'use strict'

console.log('Begin');

var module_num = 3;         // Replace this with the module number.
var lines_of_text = 28;      // Replace this with the number of lines of text TeachBot must read.
var dir = 'https://localhost:8000/';
/*******************************
 *   Setup ROS communication   *
 *******************************/
var ros = new ROSLIB.Ros({
    url: 'wss://localhost:9090'
});
ros.on('connection', function() { console.log('Connected to websocket server.');});
ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); window.alert('Error connecting to websocket server'); });
ros.on('close', function() { console.log('Connection to websocket server closed.');});

// Publishing topic
var iframe_sequence_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/bot_sequence',
    messageType: 'std_msgs/Int32'
});
var audio_duration_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/audio_duration',
    messageType: 'std_msgs/Float64'
});
var save_info_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/save_info',
    messageType: 'std_msgs/String'
});
var user_input_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/user_input',
    messageType: 'std_msgs/String'
});
// Subscribing topics
var iframe_receiver_nav = new ROSLIB.Topic({
    ros: ros,
    name: '/right_navigator_button',
    messageType: 'std_msgs/String'
})

var iframe_receiver_seq = new ROSLIB.Topic({
    ros: ros,
    name: '/iframe_sequence',
    messageType: 'std_msgs/Int32'
})
var unique_input_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/unique_input',
    messageType: 'std_msgs/String'
})

/**********************************
 *   Setup SmartSparrow SimCapi   *
 **********************************/
var simModel = new simcapi.CapiAdapter.CapiModel({
    last_interacted_obj: 'none',
    what_action: 'none'
});

/************************
 *   Setup Text Files   *
 ************************/
var text_url = dir + 'text/' + module_num + '.txt';
var textArray = new Array(lines_of_text);
jQuery.get(text_url, function(data) {
    textArray = data.split('\n');
},'text');
text.innerHTML = 'Loading...';

/*************************
 *   Setup Image Files   *
 *************************/
// Example:
// var sawyer_image_url = 'https://localhost:8000/images/Sawyer.jpg';


/*************************
 *   Setup HTML Elements   *
 *************************/
var ctx = canvas.getContext('2d');
var choice_lists = new Array(8);
for (let i=0; i<8; i++){
    choice_lists[i] = document.getElementById('choice_list' + i.toString());
    console.log(choice_lists[i])
}
var selected_choices = document.getElementById('selection_list')
/*************************
 *   Setup Audio Files   *
 *************************/
var audiofiles_mp3 = new Array(textArray.length);
var audio_duration = new Array(textArray.length);
var loaded = 0;
for(let i=0; i<audiofiles_mp3.length;i++){
    audiofiles_mp3[i] = dir + 'audio/module' + module_num + '/line' + i.toString() + '.mp3';
}

// Preload all the audio files
for (let i in audiofiles_mp3) {
    preloadAudio(audiofiles_mp3[i], i);
}
function preloadAudio(url, i) {
    let audio = new Audio();
    audio.addEventListener('canplaythrough', function() {
        audio_duration[i] = audio.duration*1000;
        loaded++;
        if (loaded == audiofiles_mp3.length){
            // All have loaded. Begin the module.
            init();
        }
    }, false);
    audio.src = url;
}

/************************
 *   Helper Functions   *
 ************************/
// Sleep for some milliseconds. Usage:
// await sleep(ms);
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// Play audio corresponding to index (e.g. smartsparrowaudio/module1/line1.mp3)
var player = document.getElementById('player');
async function play(index) {
    player.src = audiofiles_mp3[index];
    player.play();
    text.innerHTML = textArray[index];
    pub_dur(audio_duration[index]/1000.0);
    console.log('Playing audio #' + index);
}

// Publish message to ROSTopic to be received by Python
function pub_msg(index) {
    let int = new ROSLIB.Message({
        data: index
    });
    iframe_sequence_topic.publish(int);
    console.log('Sent: ' + index);
}
function pub_dur(audio_duration) {
    let msg = new ROSLIB.Message({
        data: audio_duration
    });
    //audio_duration_topic.publish(msg);
}

function save_info(string) {
    //String should be "correct", "incorrect", "time_start", "time_end"
    //"correct" and "incorrect" logs 
    let msg = new ROSLIB.Message({
        data: string
    });
    save_info_topic.publish(msg);
}


function clear_canvas(){
    ctx.clearRect(0,0, canvas.width, canvas.height);
}

function draw_rectangle(cx,cy, width, length, color){
    //ctx.clearRect(0,0,canvas.width,canvas.height);
    var corner1x = cx-width/2;
    var corner2x = cx+width/2;

    var corner1y = cy-length/2;
    var corner2y = cy+length/2;

    ctx.beginPath();    
    ctx.moveTo(corner1x,corner1y);
    ctx.lineTo(corner1x,corner2y);
    ctx.lineTo(corner2x,corner2y);
    ctx.lineTo(corner2x,corner1y);
    ctx.lineTo(corner1x,corner1y);

    ctx.lineWidth = 7;
    if(color == 'blue'){
        ctx.strokeStyle = '#3577D8';  
    }
    else if(color == 'red'){
        ctx.strokeStyle = '#C01414';
    }
    else if(color == 'green'){
        ctx.strokeStyle = '#20C014'

    }
    ctx.stroke()
    
}

function display_choicelist(){
    choice_list0.style.display = 'initial'
    choice_list1.style.display = 'initial'
    choice_list2.style.display = 'initial'
    choice_list3.style.display = 'initial'
    choice_list4.style.display = 'initial'
    choice_list5.style.display = 'initial'
    choice_list6.style.display = 'initial'
    choice_list7.style.display = 'initial'
    choice_list8.style.display = 'initial'
    selection_list.style.display = 'initial'
}

function hide_choicelist(){
    choice_list0.style.display = 'none'
    choice_list1.style.display = 'none'
    choice_list2.style.display = 'none'
    choice_list3.style.display = 'none'
    choice_list4.style.display = 'none'
    choice_list5.style.display = 'none'
    choice_list6.style.display = 'none'
    choice_list7.style.display = 'none'
    choice_list8.style.display = 'none'
    selection_list.style.display = 'none'
}
/**************************
 *   Main Functionality   *
 **************************/
 //testing waypoints pub_msg 25, counter = 30
 //obstacle course pub_msg 28 counter = 34
var counter = 0;
var wheel_value = 0;
var action_cntr = 0;
var i; //counter for for loops
var temp_string;
async function init() {
    // 1: Hello, and welcome to day three! In this module, you’re going to learn how I interface with other automated machinery on the factory floor, even when I’m not physically connected to it. 
    play(counter); 
    textdiv.style.display = 'initial';
    image_changer.style.display = 'none'
    //display_choicelist();
    await sleep(audio_duration[counter]);

    counter++;
    // 2: We’ll start with learning how I would interface with a conveyor belt, to package a box into one of the bins from the last module.
    play(counter);
    await sleep(audio_duration[counter]);

    counter++;
    // 3: Let’s say the conveyor belt is set up to deliver one empty box to me every five seconds, then stop for two seconds while I place the product in the package, then continue. So no matter what I do, the conveyor belt will do this.
    play(counter);
    await sleep(audio_duration[counter]);

    pub_msg(1);   // Advance in Py
    
    //Event Programming
    // iframe_receiver_nav.subscribe(async function(message){
    //     if(message.data.startsWith('Wheel value: ')) {
    //         console.log("entered")
    //         wheel_value = parseInt(message.data.substring(13))%choice_lists.length;
    //         for (i = 0; i < choice_lists.length; i++){
    //             choice_lists[i].style.borderColor = "#ccc"
    //         }
    //         choice_lists[wheel_value].style.borderColor = "#FFA500"
    //     }
    //     else if (message.data === "Button \'OK\': OFF"){
    //         temp_string = selected_choices.innerText;
    //         selected_choices.innerText = temp_string + "\n" + choice_lists[wheel_value].innerText;
    //         actions_ctr++;
    //     }
    //     else if (message.data === "Button \'Back\': OFF"){    
    //         if(actions_ctr == 0){
    //             return
    //         }
    //         else {
    //             temp_string = selected_choices.innerText.split("\n");
    //             temp_string.splice(temp_string.length-1, 1);
    //             selected_choices.innerText = temp_string.join("\n");
    //             actions_ctr--;
    //         }
    //     }
    //     else if (message.data === "Button \'Square\' : OFF"){
    //         let msg = new ROSLIB.Message({
    //             data: selected_choices.innerText
    //         });
    //         user_input_topic.publish(msg);
    //         iframe_receiver_nav.unsubscribe();
    //         iframe_receiver_nav.removeAllListeners();

    //     }
    // });


    // canvas_container.style.display = 'initial';
    // draw_rectangle(730, 450, 88, 38, 'blue'); //box1
    // draw_rectangle(610, 448, 38, 88, 'blue'); //box2
    // draw_rectangle(200, 400, 500, 100, 'green'); //linear slider

    // Subscribe to Python Commands
    iframe_receiver_seq.subscribe(async function(message){
        console.log('Received: ' + message.data);
        switch(message.data){

            case 1:
                counter=3;
                // 4: As long as the conveyor belt repeats this exact process every time, I have a few options for figuring out when to pick the box off the belt.
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 5: The first one we'll talk about is purely about timing. Since I know that the linear slider is doing a repetitive cycle thats constantly timed, I can just use that to figure out when to grab the box, and even when to place it back. That would look something like this.
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(2);
                break;

            case 2:
                counter++;
                // 6: This kind of methodology is known as timing based coordination.
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 7: Timing based coordination is great for things like this where I know exactly the amount of time that motions will take place. But what happens if the timing gets messed up, even by a small amount?
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 8: Something like this would happen.
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(3);
                break;

            case 3:
                counter++;
                // 9: Can you fix this by programming a different amount of time I should wait before placing the next product in the box? You can change the wait time using the scroll wheel. 
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(4);
                od_container.style.display = 'initial';
                wheel_value = 2;
                iframe_receiver_nav.subscribe(async function(message){
                    if(message.data.startsWith('Wheel value: ')){
                        wheel_value = (parseFloat(message.data.substring(13))/10)%6;
                        odometer.innerHTML = wheel_value;
                    }
                    else if(message.data === "Button \'OK\': OFF"){
                        let msg = new ROSLIB.Message({
                            data: wheel_value.toString()
                        });
                        user_input_topic.publish(msg);
                        iframe_receiver_nav.unsubscribe();
                        iframe_receiver_nav.removeAllListeners();
                    }
                })

                break;

            case 4:
                counter++;
                // 10: Seems like this one isn't working that well. Can you try again?
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(5);
                iframe_receiver_nav.subscribe(async function(message){
                    if(message.data.startsWith('Wheel value: ')){
                        wheel_value = (parseFloat(message.data.substring(13))/10)%6;
                        odometer.innerHTML = wheel_value;
                    }
                    else if(message.data === "Button \'OK\': OFF"){
                        let msg = new ROSLIB.Message({
                            data: wheel_value.toString()
                        });
                        user_input_topic.publish(msg);
                        iframe_receiver_nav.unsubscribe();
                        iframe_receiver_nav.removeAllListeners();
                    }
                })

                break;

            case 5:
                counter++;
                // 11: Hmmmm. It seems like there just isn't a perfect timing for all of the boxes being dispensed.
                play(counter);
                od_container.style.display = 'none'
                await sleep(audio_duration[counter]);

                counter++;
                // 12: This problem kind of reminds me of when I turned off my encoders and tried to move my arm to a target. Because the encoders in my arm didn’t talk to me, I had no way of telling where my arm was, and I missed the target. 
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 13: A similar problem happened here: because the conveyor belt isn’t communicating with me, I have to guess when the conveyor belt has placed a box in front of me.
                play(counter);
                await sleep(audio_duration[counter]);
                //play video?
                
                counter++;
                // 14: Just like when I “closed the loop” by listening to my encoders so that I could correct the overshoot and undershoot of my arm in module 1, I’m going to need to communicate with the conveyor belt so that I know when a box is ready for me to pick up. 
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 15: Setting up that kind of communication might take some time. For now, can you tell me when to start? 
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 16: Whenever you see the box is in the designated pickup location, press the Scroll wheel button on my arm. The designated pickup location is indicated by the red tape.
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(6);
                action_cntr = 0;
                iframe_receiver_nav.subscribe(async function(message){
                    if(message.data === "Button \'OK\': OFF"){
                        action_cntr++;
                        let msg = new ROSLIB.Message({
                            data: action_cntr.toString()
                        });
                        user_input_topic.publish(msg);
                    }
                })
                break;

            case 6:
                counter++;
                // 17: Keep it going! You're doing great!
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(7);
                break;


            case 7:
                iframe_receiver_nav.unsubscribe();
                iframe_receiver_nav.removeAllListeners();
                counter++;
                // 18: So as you can see, this kind of planning works a lot better when theres some inconsistency with the motions I'm dealing with. This kind of methodology is called event based coordination.
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 19: That said, it wouldn't be good if I relied on a human being to just tell me when it's time to move a box. 
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 20: Let me tell you about the communication I'm going to be using with the conveyor belt. Whenever an object on the conveyor belt is moved to the pick up location, the conveyor belt sends a message to me and turns on the big red light, like this.
                play(counter);
                //begin showing visual aids
                await sleep(audio_duration[counter]);

                counter++;
                // 21: After the conveyor belt sends a message to me, it waits for me to send a message back before turning of the red light and then continuing to move. With that in mind, I'm not going to send it a message again until after I've picked up the box.
                play(counter);
                //change visual aid
                await sleep(audio_duration[counter]);

                counter++;
                // 22: The whole process would look something like this.
                play(counter);
                await sleep(audio_duration[counter]);
                //turn off visual aid

                pub_msg(8);
                break;


            case 8:
                counter++;
                // 23: After watching the process, it probably looked really straight forward. In reality, I completed these different actions to complete the process. 
                play(counter);
                await sleep(audio_duration[counter]);
                //Display all the descriptions of actions

                counter++;
                // 24: Now, could you try to recreate the process with the actions I've shown? Use the scroll wheel to choose the actions. Press the Back button if you make a mistake. Press the Square button to select the process you've programmed. To test the sequence you've made, press the I button. 
                play(counter);
                //Display selector list and controls
                choice_list0.innerText = 'Move arm down';
                choice_list1.innerText = 'Move arm up';
                choice_list2.innerText = 'Move arm to conveyor pickup location';
                choice_list3.innerText = 'Open robot arm gripper';
                choice_list4.innerText = 'Close robot arm gripper';
                choice_list5.innerText = 'Move conveyor belt';
                choice_list6.innerText = 'Stop conveyor belt when box passes the light sensor';
                choice_list7.innerText = 'Move arm to drop off location';

                image_changer.style.display = 'none';
                display_choicelist();
                iframe_receiver_nav.subscribe(async function(message){
                    if(message.data.startsWith('Wheel value: ')) {
                        wheel_value = parseInt(message.data.substring(13))%choice_lists.length;
                        for (i = 0; i < choice_lists.length; i++){
                            choice_lists[i].style.borderColor = "#ccc"
                        }
                        choice_lists[wheel_value].style.borderColor = "#FFA500"
                    }
                    else if (message.data === "Button \'OK\': OFF"){
                        temp_string = selected_choices.innerText;
                        selected_choices.innerText = temp_string + "\n" + choice_lists[wheel_value].innerText;
                        action_cntr++;
                    }
                    else if (message.data === "Button \'Back\': OFF"){    
                        if(action_cntr == 0){
                            return
                        }
                        else {
                            temp_string = selected_choices.innerText.split("\n");
                            temp_string.splice(temp_string.length-1, 1);
                            selected_choices.innerText = temp_string.join("\n");
                            action_cntr--;
                        }
                    }
                    else if (message.data === "Button \'Rethink\': OFF"){
                        let msg = new ROSLIB.Message({
                            data: choice_lists[wheel_value].innerText
                        });
                        pub_msg(9);
                        await sleep(10);
                        user_input_topic.publish(msg);
                    }
                    else if (message.data === "Button \'Square\': OFF"){
                        let msg = new ROSLIB.Message({
                            data: selected_choices.innerText
                        });
                        pub_msg(10)
                        await sleep(10);
                        user_input_topic.publish(msg);
                        iframe_receiver_nav.unsubscribe();
                        iframe_receiver_nav.removeAllListeners();

                    }
                });
                break;

            case 10:
                hide_choicelist();
                counter = 24;
                // 25: That's correct! You correctly programmed that motion. I'll execute it now.
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(11);

                break;

            case 11:
                counter = 25;
                // 26: Selecting the order of actions is a form of "programming." It's how people, like you, can set instructions for robot systems to complete tasks.
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 27: Now, before we conclude this lesson, you might be wondering, "Why would I ever do timing based coordination, if event based coordination is just so much more useful?"
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 28: As a general idea, it's easier to program timing based coordination, but there are some specific instances that are great with timing based coordination. 
                play(counter);
                await sleep(audio_duration[counter]);

                counter++;
                // 29: That'll come up later on when we go over more important things in programming in later modules!
                play(counter);
                await sleep(audio_duration[counter]);

                break;

            case 12:
                counter = 28;
                // 29: Are you sure you have the right number of actions?
                play(counter);
                await sleep(audio_duration[counter]);

                counter = 31;
                // Use the scroll wheel to choose the actions. Press the Back button if you make a mistake. Press the Square button to select the process you've programmed. To see what the process looks like again, press the I button.
                play(counter);
                await sleep(audio_duration[counter]); 
                break;

            case 13:
                counter = 29;
                // 30: Your comands seem to be not quite right. Try again.
                play(counter);
                await sleep(audio_duration[counter]);

                counter = 31;
                // Use the scroll wheel to choose the actions. Press the Back button if you make a mistake. Press the Square button to select the process you've programmed. To see what the process looks like again, press the I button.
                play(counter);
                await sleep(audio_duration[counter]); 
                display_choicelist();
                iframe_receiver_nav.subscribe(async function(message){
                    if(message.data.startsWith('Wheel value: ')) {
                        wheel_value = parseInt(message.data.substring(13))%choice_lists.length;
                        for (i = 0; i < choice_lists.length; i++){
                            choice_lists[i].style.borderColor = "#ccc"
                        }
                        choice_lists[wheel_value].style.borderColor = "#FFA500"
                    }
                    else if (message.data === "Button \'OK\': OFF"){
                        temp_string = selected_choices.innerText;
                        selected_choices.innerText = temp_string + "\n" + choice_lists[wheel_value].innerText;
                        action_cntr++;
                    }
                    else if (message.data === "Button \'Back\': OFF"){    
                        if(action_cntr == 0){
                            return
                        }
                        else {
                            temp_string = selected_choices.innerText.split("\n");
                            temp_string.splice(temp_string.length-1, 1);
                            selected_choices.innerText = temp_string.join("\n");
                            action_cntr--;
                        }
                    }
                    else if (message.data === "Button \'Rethink\': OFF"){
                        let msg = new ROSLIB.Message({
                            data: choice_lists[wheel_value].innerText
                        });
                        pub_msg(9);
                        await sleep(10);
                        user_input_topic.publish(msg);
                    }
                    else if (message.data === "Button \'Square\': OFF"){
                        let msg = new ROSLIB.Message({
                            data: selected_choices.innerText
                        });
                        pub_msg(10)
                        await sleep(10);
                        user_input_topic.publish(msg);
                        iframe_receiver_nav.unsubscribe();
                        iframe_receiver_nav.removeAllListeners();

                    }
                });
                break;

            case 14:
                counter = 30;
                // 31: That action looks like this.
                await sleep(audio_duration[counter]);
                break;
            default:
                await sleep(10);
                break;

        }
    });
}