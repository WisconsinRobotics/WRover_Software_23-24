

const buttons = document.getElementsByClassName('launch-button');
const competitionButton = buttons[0];
const scienceButton = buttons[1];
const deliveryButton = buttons[2];
const serviceButton = buttons[3];


competitionButton.addEventListener('click', () => {
    console.log('clicked');
    fetch('/startup/competitionMission.py', {
        method: 'GET',
        headers: {
            'Content-Type': 'application/json'
        },
    })
    .then(response => response.json())
    .then(data => {
        console.log(data);
    })
});

scienceButton.addEventListener('click', () => {
    console.log('clicked');
    fetch('/startup/scienceMission.py', {
        method: 'GET',
        headers: {
            'Content-Type': 'application/json'
        },
    })
    .then(response => response.json())
    .then(data => {
        console.log(data);
    })
});

deliveryButton.addEventListener('click', () => {
    console.log('clicked');
    fetch('/startup/deliveryMission.py', {
        method: 'GET',
        headers: {
            'Content-Type': 'application/json'
        },
    })
    .then(response => response.json())
    .then(data => {
        console.log(data);
    })
});

serviceButton.addEventListener('click', () => {
    console.log('clicked');
    fetch('/startup/serviceMission.py', {
        method: 'GET',
        headers: {
            'Content-Type': 'application/json'
        },
    })
    .then(response => response.json())
    .then(data => {
        console.log(data);
    })
});
