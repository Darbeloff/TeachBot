'use strict'

const module_num = 2;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [image,canvas_container]);
var cw = m.cw;
var ch = m.ch;

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = '1Dof';
async function main() {
    m.displayOff();
    canvas_container.style.display = 'initial';
    m.start([start_seq,0]);
}