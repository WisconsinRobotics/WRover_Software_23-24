let canvas1  = document.getElementById('video1');
let zoomingDiv1 = document.getElementById('zoomingDiv1');
let canvas2  = document.getElementById('video2');
let zoomingDiv2 = document.getElementById('zoomingDiv2');
let canvas3  = document.getElementById('video3');
let zoomingDiv3 = document.getElementById('zoomingDiv3');
let canvas4  = document.getElementById('video4');
let zoomingDiv4 = document.getElementById('zoomingDiv4');
let ctx1 = canvas1.getContext('2d');
let ctx2 = canvas2.getContext('2d');
let ctx3 = canvas3.getContext('2d');
let ctx4 = canvas4.getContext('2d');
let img = new Image();

// Connect to the WebSocket server
let socket = new WebSocket("ws://localhost:8081"); // Replace with your WS proxy server
let socket2 = new WebSocket("ws://localhost:8082"); // Replace with your WS proxy server
let socket3 = new WebSocket("ws://localhost:8083"); // Replace with your WS proxy server
let socket4 = new WebSocket("ws://localhost:8084"); // Replace with your WS proxy server

socket.onmessage = function(event) {
    if(zoomingDiv1.style.display == 'none'){
        return;
    }
    img.src = 'data:image/jpeg;base64,' + event.data;

    img.onload = function() {
        ctx1.drawImage(img, 0, 0, canvas1.width, canvas1.height);
    };
};

socket2.onmessage = function(event) {
    if(zoomingDiv2.style.display == 'none'){
        return;
    }
    img.src = 'data:image/jpeg;base64,' + event.data;

    img.onload = function() {
        ctx2.drawImage(img, 0, 0, canvas2.width, canvas2.height);
    };
};

socket3.onmessage = function(event) {
    if(zoomingDiv3.style.display == 'none'){
        return;
    }
    img.src = 'data:image/jpeg;base64,' + event.data;

    img.onload = function() {
        ctx3.drawImage(img, 0, 0, canvas3.width, canvas3.height);
    };
}

socket4.onmessage = function(event) {
    if(zoomingDiv4.style.display == 'none'){
        return;
    }
    img.src = 'data:image/jpeg;base64,' + event.data;

    img.onload = function() {
        ctx4.drawImage(img, 0, 0, canvas4.width, canvas4.height);
    };
};