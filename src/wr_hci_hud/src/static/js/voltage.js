const voltageDiv = document.getElementById('voltage-div');
console.log(voltageDiv);
let curr_voltage = 5;
let max_voltage = 10;

let currentPercentage = curr_voltage / max_voltage * 100;

voltageDiv.setAttribute('style', `background-image: linear-gradient(180deg, rgb(42, 42, 42) ${100 - currentPercentage}%, #00ff44 0%)`)