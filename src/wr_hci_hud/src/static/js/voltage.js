const ampereDiv = document.getElementById('ampere-div');
const batteryDiv = document.getElementById('battery-div');

let curr_ampere = 0.5;
let max_ampere = 10;

let curr_battery = 0.5;
let max_battery = 10;

lowThreshold = 20;

let currentAmperePercentage = curr_ampere / max_ampere * 100;
let currentBatteryPercentage = curr_battery / max_battery * 100;
//100 milliseconds

const green = '#00ff44';
const red = '#ff0000';

let currAmpereColor = green;
let currBatteryColor = green;

setInterval(() => {
    // curr_ampere = Math.random();
    // curr_battery = Math.random();
    
    currentAmperePercentage = curr_ampere / max_ampere * 100;
    currentBatteryPercentage = curr_battery / max_battery * 100;
   
   
    if(currentBatteryPercentage < lowThreshold) {
        batteryDiv.innerHTML = 'Low Battery';
        currBatteryColor = red;
    } else{
        batteryDiv.innerHTML = Math.round(currentBatteryPercentage) + "%";
        currBatteryColor = green;
    }

    if(currentAmperePercentage < lowThreshold) {
        ampereDiv.innerHTML = 'Low Ampere';
        currAmpereColor = red;
    } else{
        ampereDiv.innerHTML = curr_ampere.toFixed(1) + "A";
        currAmpereColor = green;
    }
    
    ampereDiv.setAttribute('style', `background-image: linear-gradient(-180deg, rgb(42, 42, 42) ${100 - currentAmperePercentage}%, ${currAmpereColor} 0%)`)
    batteryDiv.setAttribute('style', `background-image: linear-gradient(-180deg, rgb(42, 42, 42) ${100 - currentBatteryPercentage}%, ${currBatteryColor} 0%)`)
}, 100)



//Ampere meter