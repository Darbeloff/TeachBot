'use strict'

const module_num = 1;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
<<<<<<< HEAD
var m = new Module(module_num, main, [canvas_obj, image, animator]);
=======
>>>>>>> origin

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = '2dPosition';
var start_location = 22;
async function main() {
    m.displayOff();
    m.set_graphic_mode({mode:'image'});
    m.start([start_seq,start_location]);
}