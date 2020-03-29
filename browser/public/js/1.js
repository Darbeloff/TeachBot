'use strict'

const module_num = 1;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [image, animator]);
var cw = m.cw;
var ch = m.ch;

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = 'intro';
async function main() {
    m.displayOff();
    m.start([start_seq,0]);
}