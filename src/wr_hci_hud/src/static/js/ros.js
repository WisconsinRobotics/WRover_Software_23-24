let rosConnected = false;

var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
});


ros.on('connection', function() {
  console.log('Connected to websocket server.');
  diagnosticInput.value += 'Connected to websocket server.\n';
  rosConnected = true;
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
  diagnosticInput.value += 'Error connecting to websocket server: ' + error + '\n';
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
  diagnosticInput.value += 'Connection to websocket server closed.\n';
});

var reconnectIntervalId = setInterval(function() {
  if(!rosConnected) {
    console.log('Trying to reconnect to websocket server.');
    diagnosticInput.value += 'Trying to reconnect to websocket server.\n';
    ros.connect('ws://localhost:9090');
  }

  if(rosConnected) {
    clearInterval(reconnectIntervalId); 
  }
}, 1000);

// var streamListener = new ROSLIB.Topic({
//   ros : ros,
//   name : '/video',
//   messageType : 'sensor_msgs/CompressedImage'
// });

// streamListener.subscribe(function(message) {
//   video1.src = 'data:image/jpeg;base64,' + imageData;
// });

// setInterval(function() {
//   listener.publish(new ROSLIB.Message({
//     data : 'path/to/your/jpeg/file.jpg'
//   }));
// }, 5000);

// Publishing a Topic
// ------------------

// var cmdVel = new ROSLIB.Topic({
//   ros : ros,
//   name : '/cmd_vel',
//   messageType : 'geometry_msgs/Twist'
// });

// var twist = new ROSLIB.Message({
//   linear : {
//     x : 0.1,
//     y : 0.2,
//     z : 0.3
//   },
//   angular : {
//     x : -0.1,
//     y : -0.2,
//     z : -0.3
//   }
// });
// cmdVel.publish(twist);

// Subscribing to a Topic
// ----------------------
var listener = new ROSLIB.Topic({
  ros : ros,
  name : '/listener',
  messageType : 'std_msgs/String'
});

listener.subscribe(function(message) {
  console.log('Received message on ' + listener.name + ': ' + message.data);
  diagnosticInput.value += 'Received message on ' + listener.name + ': ' + message.data + '\n';
});

setInterval(function() {
  listener.publish(new ROSLIB.Message({
    data : 'Hello World!'
  }));
}, 5000);

var voltageListener = new ROSLIB.Topic({
  ros: ros,
  name: '/voltage',
  messageType: 'std_msgs/Float32'
});

voltageListener.subscribe(function(message) {
  // console.log('Received voltage: ' + message.data);
  // diagnosticInput.value += 'Received voltage: ' + message.data + '\n';
  curr_battery = message.data;
});

setInterval(function() {
  voltageListener.publish(new ROSLIB.Message({
    data: Math.round(Math.random()*10, 2)
  }));
}, 1000);

var ampereListener = new ROSLIB.Topic({
  ros: ros,
  name: '/ampere',
  messageType: 'std_msgs/Float32'
});

ampereListener.subscribe(function(message) {
  // console.log('Received ampere: ' + message.data);
  // diagnosticInput.value += 'Received ampere: ' + message.data + '\n';
  curr_ampere = message.data;
});

setInterval(function() {
  ampereListener.publish(new ROSLIB.Message({
    data: Math.round(Math.random()*10, 2)
  }));
}, 1000);

var xCoordinatesListener = new ROSLIB.Topic({
  ros: ros,
  name: '/xcoordinates',
  messageType: 'std_msgs/Float32'
});

xCoordinatesListener.subscribe(function(message) {
  // console.log('Received x coordinates: ' + message.data);
  // diagnosticInput.value += 'Received x coordinates: ' + message.data + '\n';
  xcoord = message.data;
});

setInterval(function() {
  xCoordinatesListener.publish(new ROSLIB.Message({
    data: Math.round(Math.random()*10, 2)
  }));
}, 1000);

var yCoordinatesListener = new ROSLIB.Topic({
  ros: ros,
  name: '/ycoordinates',
  messageType: 'std_msgs/Float32'
});

yCoordinatesListener.subscribe(function(message) {
  // console.log('Received y coordinates: ' + message.data);
  // diagnosticInput.value += 'Received y coordinates: ' + message.data + '\n';
  ycoord = message.data;
});

setInterval(function() {
  yCoordinatesListener.publish(new ROSLIB.Message({
    data: Math.round(Math.random()*10, 2)
  }));
}, 1000);

var xVelocityListener = new ROSLIB.Topic({
  ros: ros,
  name: '/xvelocity',
  messageType: 'std_msgs/Float32'
});

xVelocityListener.subscribe(function(message) {
  // console.log('Received x velocity: ' + message.data);
  // diagnosticInput.value += 'Received x velocity: ' + message.data + '\n';
  xvelocity = message.data;
});

setInterval(function() {
  xVelocityListener.publish(new ROSLIB.Message({
    data: Math.round(Math.random()*10, 2)
  }));
}, 1000);

var yVelocityListener = new ROSLIB.Topic({
  ros: ros,
  name: '/yvelocity',
  messageType: 'std_msgs/Float32'
});

yVelocityListener.subscribe(function(message) {
  // console.log('Received y velocity: ' + message.data);
  // diagnosticInput.value += 'Received y velocity: ' + message.data + '\n';
  yvelocity = message.data;
});

setInterval(function() {
  yVelocityListener.publish(new ROSLIB.Message({
    data: Math.round(Math.random()*10, 2)
  }));
}, 1000);

var accelListener = new ROSLIB.Topic({
  ros: ros,
  name: '/acceleration',
  messageType: 'std_msgs/Float32'
});

accelListener.subscribe(function(message) {
  // console.log('Received acceleration: ' + message.data);
  // diagnosticInput.value += 'Received acceleration: ' + message.data + '\n';
  accel = message.data;
});

setInterval(function() {
  accelListener.publish(new ROSLIB.Message({
    data: Math.round(Math.random()*10, 2)
  }));
}, 1000);

var distanceListener = new ROSLIB.Topic({
  ros: ros,
  name: '/distance',
  messageType: 'std_msgs/Float32'
});

distanceListener.subscribe(function(message) {
  // console.log('Received distance: ' + message.data);
  // diagnosticInput.value += 'Received distance: ' + message.data + '\n';
  distance = message.data;
});

setInterval(function() {
  distanceListener.publish(new ROSLIB.Message({
    data: Math.round(Math.random()*10, 2)
  }));
}, 1000);


var joystickListener = new ROSLIB.Topic({
  ros: ros,
  name: '/joystick',
  messageType: 'std_msgs/Float32MultiArray'
});

joystickListener.subscribe(function(message) {
  // console.log('Received joystick: ' + message.data);
  diagnosticInput.value += 'Received joystick: ' + message.data + '\n';
  updateJoystick(message.data[0], message.data[1]);
});

setInterval(function() {
  joystickListener.publish(new ROSLIB.Message({
    data: [Math.round(Math.random()*20-10, 2), Math.round(Math.random()*20-10, 2)]
  }));
}, 1000);