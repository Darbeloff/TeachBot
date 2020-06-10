'use strict'

var module_name = window.location.href.substring(DIR.length+1);
console.log(module_name);

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_name, main);

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = 'intro';
var start_location = 0;

async function main() {
	console.log('Loaded')
	m.displayOff();
	m.set_graphic_mode({mode:'image'});
	m.start([start_seq,start_location]);
}