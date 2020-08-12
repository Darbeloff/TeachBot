'use strict'
const module_num = 1;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [image, animator, canvas_obj]);

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = 'encoders';
var start_location = 32;

async function main() {
    m.displayOff();
    m.set_graphic_mode({mode:'image'});
    m.start([start_seq,start_location]);
}