// Get references to the buttons and diagnostic input
const launchButton = document.querySelectorAll('.launch-button');
const diagnosticInput = document.querySelector('.diagnostic-input');
let startTime;
let storedText = '';
let intervalId; // Variable to store the interval ID
let detectScroll = false;
let autoscroll = false;

var options = {
    zone: document.getElementById('joystick1'),
    color: "white",
    // dataOnly: Boolean,              // no dom element whatsoever
    position: {top: "20%", right: "40%"},               // preset position for 'static' mode
    mode: "static",                   // 'dynamic', 'static' or 'semi'
    // restJoystick: Boolean|Object,   // Re-center joystick on rest state
    restOpacity: 1,            // opacity when not 'dynamic' and rested
    // lockX: Boolean,                 // only move on the X axis
    // lockY: Boolean,                 // only move on the Y axis
};

var manager = nipplejs.create(options);

var joystick = manager.get(0);
joystick.hide();
console.log(joystick);
joystick.setPosition(() => console.log(""), 1000, 1000);
joystick.lockx = true;
joystick.locky = true;

// Add event listeners to the buttons
for (let i = 0; i < launchButton.length; i++) {
    launchButton[i].addEventListener('click', handleButtonClick);
}

// Add event listener for scrolling in diagnostic input
diagnosticInput.addEventListener('scroll', handleScroll);

// Define the event listener function for button click
function handleButtonClick(event) {
    if (!confirm("Are you sure you want to continue?")) {
        return;
    }
    
    diagnosticInput.value = '';
    clearInterval(intervalId); // Clear the interval

    // Check if the clicked button already has the 'clicked-button' class
    if (event.target.classList.contains('clicked-button')) {
        // Remove the 'clicked-button' class and reset the button text
        event.target.classList.remove('clicked-button');
        event.target.textContent = storedText;
        return;
    }

    // Remove the 'clicked-button' class from all buttons
    for (let i = 0; i < launchButton.length; i++) {
        if (launchButton[i].classList.contains('clicked-button')) {
            launchButton[i].classList.remove('clicked-button');
            launchButton[i].textContent = storedText;
        }
    }

    diagnosticInput.value = "Launching " + event.target.textContent + "..." + "\n";

    // Store the text of the clicked button
    storedText = event.target.textContent;

    // Add the 'clicked-button' class to the clicked button
    event.target.classList.add('clicked-button');
    event.target.textContent = 'Stop';

    // Start the timer
    startTime = new Date().getTime();

    // Update the diagnostic input every second
    intervalId = setInterval(() => {
        const elapsedTime = Math.floor((new Date().getTime() - startTime) / 1000);
        diagnosticInput.value += `Elapsed time: ${elapsedTime} s\n`;
        if(!detectScroll) {
            diagnosticInput.scrollTop = diagnosticInput.scrollHeight; // Scroll to the bottom
            autoscroll = true;
        }
    }, 1000);
}

// Define the event listener function for scrolling
function handleScroll() {
    if(autoscroll) {
        autoscroll = false;
        return;
    }

    if (diagnosticInput.scrollTop - (diagnosticInput.scrollHeight - diagnosticInput.clientHeight) > 0) {
        detectScroll = false;
        return;
    }

    detectScroll = true;
}

