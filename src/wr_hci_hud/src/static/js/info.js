const xcoordBox = document.getElementById('xcoord');
const ycoordBox = document.getElementById('ycoord');
const xvelocityBox = document.getElementById('xvelocity');
const yvelocityBox = document.getElementById('yvelocity');
const accelBox = document.getElementById('accel');
const distanceBox = document.getElementById('distance');

let xcoord = 0;
let ycoord = 0;
let xvelocity = 0;
let yvelocity = 0;
let accel = 0;
let distance = 0;

setInterval(() => {

    xcoordBox.innerHTML = "X: " + xcoord.toFixed(2);
    ycoordBox.innerHTML = "Y: " + ycoord.toFixed(2);
    xvelocityBox.innerHTML = "X: " + xvelocity.toFixed(2) + " m/s";
    yvelocityBox.innerHTML = "Y: " + yvelocity.toFixed(2) + " m/s";
    accelBox.innerHTML = accel.toFixed(2) + " m/s^2";
    distanceBox.innerHTML = distance.toFixed(2) + " m";
}, 500);