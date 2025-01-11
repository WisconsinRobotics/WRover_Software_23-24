// Get references to the buttons and diagnostic input
const launchButton = document.querySelectorAll('.launch-button');
const diagnosticInput = document.querySelector('.diagnostic-input');
let startTime;
let storedText = '';
let intervalId; // Variable to store the interval ID
let detectScroll = false;
let autoscroll = false;

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
    // clearInterval(intervalId);

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

    diagnosticInput.value = "Launching " + event.target.textContent + "...\n";

    // Store the text of the clicked button
    storedText = event.target.textContent;

    // Add the 'clicked-button' class to the clicked button
    event.target.classList.add('clicked-button');
    event.target.textContent = 'Stop';

    // Start the timer
    // startTime = new Date().getTime();

    // Update the diagnostic input every second
}

intervalId = setInterval(() => {
    // const elapsedTime = Math.floor((new Date().getTime() - startTime) / 1000);
    // diagnosticInput.value += `Elapsed time: ${elapsedTime} s\n`;
    if(!detectScroll) {
        diagnosticInput.scrollTop = diagnosticInput.scrollHeight; // Scroll to the bottom
        autoscroll = true;
    }
}, 100);


// Define the event listener function for scrolling
function handleScroll() {
    if(autoscroll) {
        autoscroll = false;
        return;
    }

    if (diagnosticInput.scrollTop - (diagnosticInput.scrollHeight - diagnosticInput.clientHeight) > -2) {
        // console.log('Resume autoscrolling');
        console.log(diagnosticInput.scrollTop - (diagnosticInput.scrollHeight - diagnosticInput.clientHeight));
        detectScroll = false;
        return;
    }
    // console.log('Stop autoscrolling');
    detectScroll = true;
}

