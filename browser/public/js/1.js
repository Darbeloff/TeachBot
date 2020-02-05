'use strict'

const module_num = 1;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [canvas_obj, image, animator]);

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = 'intro';
var start_location = 0;
async function main() {
    m.displayOff();
    image.style.display = 'initial';
    m.start([start_seq,start_location]);
}