#!/usr/bin/env node

/**
 * Module dependencies.
 */
const rosnodejs = require('rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;

var http_app = require('./app');
var https_app = require('./app');
var debug = require('debug')('teachbot:server');
var http = require('http');
var https = require('https');
var fs = require('fs');
var privateKey = fs.readFileSync('sslcert/ia2.key', 'utf8');
var certificate = fs.readFileSync('sslcert/ia2.crt',  'utf8');
var credentials = {key: privateKey, cert: certificate};
/**
 * Get port from environment and store in Express.
 */

var http_port = normalizePort(process.env.PORT || '3000');
var https_port = normalizePort(process.env.PORT || '8000');
http_app.set('port', http_port);
https_app.set('port', https_port);
/**
 * Create HTTP server.
 */

var http_server = http.createServer(http_app);
var https_server = https.createServer(credentials, https_app);
/**
 * Listen on provided port, on all network interfaces.
 */

http_server.listen(http_port);
http_server.on('error', onError);
http_server.on('listening', http_onListening);

https_server.listen(https_port);
https_server.on('error', onError);
https_server.on('listening', https_onListening);

listener();
/** 
* Handle ROS communication
*/
function listener() {
  // Register node with ROS master
  rosnodejs.initNode('/www_node')
    .then((rosNode) => {
      // Create ROS subscriber on the 'chatter' topic expecting String messages
      let sub = rosNode.subscribe('/www_comm', std_msgs.String,
        (data) => { // define callback execution
          rosnodejs.log.info('I heard: [' + data.data + ']');

        }
      );
    });
}
/**
 * Normalize a port into a number, string, or false.
 */

function normalizePort(val) {
  var port = parseInt(val, 10);

  if (isNaN(port)) {
    // named pipe
    return val;
  }

  if (port >= 0) {
    // port number
    return port;
  }

  return false;
}

/**
 * Event listener for HTTP server "error" event.
 */

function onError(error) {
  if (error.syscall !== 'listen') {
    throw error;
  }

  var bind = typeof https_port === 'string'
    ? 'Pipe ' + https_port
    : 'Port ' + https_port;

  // handle specific listen errors with friendly messages
  switch (error.code) {
    case 'EACCES':
      console.error(bind + ' requires elevated privileges');
      process.exit(1);
      break;
    case 'EADDRINUSE':
      console.error(bind + ' is already in use');
      process.exit(1);
      break;
    default:
      throw error;
  }
}

/**
 * Event listener for HTTP server "listening" event.
 */

function http_onListening() {
  var addr = http_server.address();
  var bind = typeof addr === 'string'
    ? 'pipe ' + addr
    : 'port ' + addr.port;
  debug('Listening on ' + bind);
}

function https_onListening() {
  var addr = https_server.address();
  var bind = typeof addr === 'string'
    ? 'pipe ' + addr
    : 'port ' + addr.port;
  debug('Listening on ' + bind);
}