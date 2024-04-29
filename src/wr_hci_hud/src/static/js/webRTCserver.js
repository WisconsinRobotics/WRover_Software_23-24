// This is the node.js server application
// import { WebSocketServer } from '../../../node_modules/ws/lib/websocket-server.js';
// import { readFile } from '../../../node_modules/fs/lib/fs.js';
// import { createServer } from '../../../node_modules/http/lib/http.js';
const WebSocket = require('ws');

var fs = require('fs');
var http = require('http');

var clients = []; 

// const wss = new WebSocket.Server({ port: 8080 });

// change key and cert if you have other ones you use with a different name
var server = http.createServer(function(request, response) {
    // fs.readFile(__dirname + '/index.html',
    // function (err, data) {
    //     if (err) {
    //         response.writeHead(500);
    //         return response.end('Error loading index.html');
    //     }
    //     response.writeHead(200);
    //     response.end(data);
    // });
    response.writeHead(200, { 'Content-Type': 'text/plain' });
    response.end('Hello World!\n');
});

server.listen(8080, function() {
  console.log((new Date()) + " Server is listening on port 8080");
});

// create the server
wsServer = new WebSocket.Server({
  server: server
});

function sendCallback(err) {
  if (err) console.error("send() error: " + err);
}

// This callback function is called every time someone
// tries to connect to the WebSocket server


wsServer.on('connection', function(connection) {
    console.log((new Date()) + ' Connection accepted.');
//   console.log(' Connection ' + connection._socket.remoteAddress);
  clients.push(connection);
    
  // This is the most important callback for us, we'll handle
  // all messages from users here.
  connection.on('message', function(message) {
    console.log(typeof message);
    // if (message.type === 'utf8') {
      // process WebSocket message
      console.log((new Date()) + ' Received Message ' + message.utf8Data);
      // broadcast message to all connected clients
      clients.forEach(function (outputConnection) {
        // if (outputConnection != connection) {
            console.log('sending message to client');
          outputConnection.send(message.toString(), sendCallback);
        // }   
      }); 
    // }   
  }); 
    
  connection.on('close', function(connection) {
    // close user connection
    console.log((new Date()) + " Peer disconnected.");    
  }); 
});
