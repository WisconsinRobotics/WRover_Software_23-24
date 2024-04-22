const joystickDiv = document.getElementById('joystick');

const joystickBase = document.createElement('div');
joystickBase.setAttribute('id', 'joystickBase')

const joystickHandle = document.createElement('div');
joystickHandle.setAttribute('id', 'joystickHandle')

joystickDiv.appendChild(joystickBase);
joystickBase.appendChild(joystickHandle);

// let joystickx = 0;
// let joystickY = 0;

function updateJoystick(joystickX, joystickY) {
    // joystickHandle.style.transform = "none";
    joystickHandle.style.transform = "translate(" + joystickX*5 + "px, " + joystickY*5 + "px)";
}
