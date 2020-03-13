'use strict'

const module_num = 'test';         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [canvas_obj]);

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = 'intro';
var start_location = 0;
async function main() {
    // image.style.display = 'initial';
    m.start([start_seq,start_location]);
}