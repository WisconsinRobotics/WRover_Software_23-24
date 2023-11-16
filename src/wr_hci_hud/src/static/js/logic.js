
// Get references to the buttons and diagnostic input
const launchButton = document.querySelectorAll('.launch-button');
const diagnosticInput = document.querySelector('.diagnostic-input');
let startTime;

// Add event listeners to the buttons
for (let i = 0; i < launchButton.length; i++) {
    launchButton[i].addEventListener('click', handleButtonClick);
}

// Define the event listener function
function handleButtonClick(event) {
    if (!confirm("Are you sure you want to continue?")) {
        return
    }

    // Remove the clicked-button class from all buttons
    for (let i = 0; i < launchButton.length; i++) {
        if(launchButton[i].classList.contains('clicked-button')){
                launchButton[i].classList.remove('clicked-button');
        }
    }

    // Add the clicked-button class to the clicked button
    event.target.classList.add('clicked-button');
    console.log(event.target.id)

    // Start the timer
    startTime = new Date().getTime();

    // Update the diagnostic input every second
    setInterval(() => {
        const elapsedTime = Math.floor((new Date().getTime() - startTime) / 1000);
        diagnosticInput.value = `Elapsed time: ${elapsedTime} s`;
    }, 1000);
}




