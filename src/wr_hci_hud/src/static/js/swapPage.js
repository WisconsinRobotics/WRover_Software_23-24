const launch = document.getElementById('launchHeading');
const data = document.getElementById('dataHeading');
const motor = document.getElementById('motorHeading');
const info = document.getElementById('infoHeading');
const stat = document.getElementById('statusHeading');

const launchPage = document.getElementById('launchPage');
const dataPage = document.getElementById('dataPage');
const motorPage = document.getElementById('motorPage');
const diagnosticPage = document.getElementById('info-box');
const statusPage = document.getElementById('status-page');

const cam1 = document.getElementById('cam1');
const cam2 = document.getElementById('cam2');
const cam3 = document.getElementById('cam3');
const cam4 = document.getElementById('cam4');
const camMulti = document.getElementById('camMulti');


const video1 = document.getElementById('video1');
const video2 = document.getElementById('video2');
const video3 = document.getElementById('video3');
const video4 = document.getElementById('video4');

const video1Div = document.getElementById('zoomingDiv1');
const video2Div = document.getElementById('zoomingDiv2');
const video3Div = document.getElementById('zoomingDiv3');
const video4Div = document.getElementById('zoomingDiv4');

video2Div.style.display = 'none';
video3Div.style.display = 'none';
video4Div.style.display = 'none';


const slide = document.getElementsByClassName('slidecontainer')[0];
const slide2 = document.getElementsByClassName('slidecontainer')[1];
const slide3 = document.getElementsByClassName('slidecontainer')[2];
const slide4 = document.getElementsByClassName('slidecontainer')[3];

const enlarge = document.getElementById('vidEnlarge');

const videoSquare = document.getElementsByClassName('Video')[0];

launch.addEventListener('click', () => {
    launchPage.style.display = 'block';
    dataPage.style.display = 'none';
    motorPage.style.display = 'none';
    data.classList.remove('current-heading');
    motor.classList.remove('current-heading');
    launch.classList.add('current-heading');
});

data.addEventListener('click', () => {
    launchPage.style.display = 'none';
    motorPage.style.display = 'none';
    dataPage.style.display = 'block';
    launch.classList.remove('current-heading');
    motor.classList.remove('current-heading');
    data.classList.add('current-heading');
});

motor.addEventListener('click', () => {
    launchPage.style.display = 'none';
    dataPage.style.display = 'none';
    motorPage.style.display = 'block';
    launch.classList.remove('current-heading');
    data.classList.remove('current-heading');
    motor.classList.add('current-heading');
});

info.addEventListener('click', () => {
    diagnosticPage.style.display = 'block';
    statusPage.style.display = 'none';
    stat.classList.remove('current-heading');
    info.classList.add('current-heading');
});

stat.addEventListener('click', () => {
    diagnosticPage.style.display = 'none';
    statusPage.style.display = 'block';
    info.classList.remove('current-heading');
    stat.classList.add('current-heading');
});


cam1.addEventListener('click', () => {
    video1Div.style.display = 'block';
    video2Div.style.display = 'none';
    video3Div.style.display = 'none';
    video4Div.style.display = 'none';
    camMulti.classList.remove('current-heading');
    cam4.classList.remove('current-heading');
    cam3.classList.remove('current-heading');
    cam2.classList.remove('current-heading');
    cam1.classList.add('current-heading');
    removeMultiVideoClasses();
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        slide.classList.remove('hideSlide');
        slide2.classList.add('hideSlide');
        slide3.classList.add('hideSlide');
        slide4.classList.add('hideSlide');
    }
});

cam2.addEventListener('click', () => {
    video1Div.style.display = 'none';
    video2Div.style.display = 'block';
    video3Div.style.display = 'none';
    video4Div.style.display = 'none';
    camMulti.classList.remove('current-heading');
    cam4.classList.remove('current-heading');
    cam3.classList.remove('current-heading');
    cam1.classList.remove('current-heading');
    cam2.classList.add('current-heading');
    removeMultiVideoClasses();
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        slide.classList.add('hideSlide');
        slide2.classList.remove('hideSlide');
        slide3.classList.add('hideSlide');
        slide4.classList.add('hideSlide');
    }
});

cam3.addEventListener('click', () => {
    video1Div.style.display = 'none';
    video2Div.style.display = 'none';
    video3Div.style.display = 'block';
    video4Div.style.display = 'none';
    cam1.classList.remove('current-heading');
    cam2.classList.remove('current-heading');
    cam3.classList.add('current-heading');
    cam4.classList.remove('current-heading');
    camMulti.classList.remove('current-heading');
    removeMultiVideoClasses();
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        slide.classList.add('hideSlide');
        slide2.classList.add('hideSlide');
        slide3.classList.remove('hideSlide');
        slide4.classList.add('hideSlide');
    }
    
});

cam4.addEventListener('click', () => {
    video1Div.style.display = 'none';
    video2Div.style.display = 'none';
    video3Div.style.display = 'none';
    video4Div.style.display = 'block';
    cam1.classList.remove('current-heading');
    cam2.classList.remove('current-heading');
    cam3.classList.remove('current-heading');
    camMulti.classList.remove('current-heading');
    cam4.classList.add('current-heading');
    removeMultiVideoClasses();
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        slide.classList.add('hideSlide');
        slide2.classList.add('hideSlide');
        slide3.classList.add('hideSlide');
        slide4.classList.remove('hideSlide');
    }
});

camMulti.addEventListener('click', () => {
    video1Div.style.display = 'block';
    video2Div.style.display = 'block';
    video3Div.style.display = 'block';
    video4Div.style.display = 'block';
    cam1.classList.remove('current-heading');
    cam2.classList.remove('current-heading');
    cam3.classList.remove('current-heading');
    cam4.classList.remove('current-heading');
    camMulti.classList.add('current-heading');
    video1.classList.add('multiVideo');
    video2.classList.add('multiVideo');
    video3.classList.add('multiVideo');
    video4.classList.add('multiVideo');
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        video1.classList.add('enlargeMultiVideo');
        video2.classList.add('enlargeMultiVideo');
        video3.classList.add('enlargeMultiVideo');
        video4.classList.add('enlargeMultiVideo');
    }
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        slide.classList.add('hideSlide');
        slide2.classList.add('hideSlide');
        slide3.classList.add('hideSlide');
        slide4.classList.add('hideSlide');
    }
});

let logo = document.getElementById('logo');

enlarge.addEventListener('click', () => {
    logo.toggleAttribute('hidden');

    videoSquare.classList.toggle('enlargedVideoWrapper');
    video1.classList.toggle('enlargedVideo');
    video2.classList.toggle('enlargedVideo');
    video3.classList.toggle('enlargedVideo');
    video4.classList.toggle('enlargedVideo');
    if(cam1.classList.contains('current-heading')) {
        slide.classList.toggle('hideSlide');
    } else if(cam2.classList.contains('current-heading')) {
        slide2.classList.toggle('hideSlide');
    } else if(cam3.classList.contains('current-heading')) {
        slide3.classList.toggle('hideSlide');
    } else if(cam4.classList.contains('current-heading')) {
        slide4.classList.toggle('hideSlide');
    } else if(camMulti.classList.contains('current-heading')) {
        video1.classList.toggle('enlargeMultiVideo');
        video2.classList.toggle('enlargeMultiVideo');
        video3.classList.toggle('enlargeMultiVideo');
        video4.classList.toggle('enlargeMultiVideo');
    }

    if(enlarge.innerHTML === '^') {
        enlarge.innerHTML = '-';
    } else {
        enlarge.innerHTML = '^';
    }
});

function removeMultiVideoClasses() {
    video1.classList.remove('enlargeMultiVideo');
    video2.classList.remove('enlargeMultiVideo');
    video3.classList.remove('enlargeMultiVideo');
    video4.classList.remove('enlargeMultiVideo');
    video1.classList.remove('multiVideo');
    video2.classList.remove('multiVideo');
    video3.classList.remove('multiVideo');
    video4.classList.remove('multiVideo');
}
