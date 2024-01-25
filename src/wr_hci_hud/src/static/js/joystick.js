const joystickDiv = document.getElementById('joystick');

const joystickBase = document.createElement('div');
joystickBase.setAttribute('id', 'joystickBase')

const joystickHandle = document.createElement('div');
joystickHandle.setAttribute('id', 'joystickHandle')

joystickDiv.appendChild(joystickBase);
joystickBase.appendChild(joystickHandle);

let inputX = 0;
let inputY = 0;

// Add animation to joystickBase based on inputX and inputY
function updateJoystickAnimation() {
    joystickHandle.style.transform = `translate(${inputX}px, ${inputY}px)`;
    joystickHandle.innerHTML = `X: ${inputX}\n Y: ${inputY}`;
}

// Call the updateJoystickAnimation function whenever inputX or inputY changes
function handleInputUpdate() {
    updateJoystickAnimation();
}

// Example usage: update inputX and inputY values
inputX = 10;
inputY = -10;
handleInputUpdate();

joystickHandle.addEventListener('mousedown', (event) => {
    let currX = event.clientX;
    let currY = event.clientY;

    function handleMouseMove(event) {
        inputX += event.clientX - currX;
        inputY += event.clientY - currY;
        currX = event.clientX;
        currY = event.clientY;
        handleInputUpdate();
    }
    joystickHandle.addEventListener('mousemove', handleMouseMove);
    joystickHandle.addEventListener('mouseup', (event) => {
        joystickHandle.removeEventListener('mousemove', handleMouseMove);
    });

    joystickHandle.addEventListener('mouseleave', (event) => {
        joystickHandle.removeEventListener('mousemove', handleMouseMove);
    });
    
});
