const launch = document.getElementById('launchHeading');
const data = document.getElementById('dataHeading');

const cam1 = document.getElementById('cam1');
const cam2 = document.getElementById('cam2');
const cam3 = document.getElementById('cam3');
const camMulti = document.getElementById('camMulti');

const launchPage = document.getElementById('launchPage');
const dataPage = document.getElementById('dataPage');

const video1 = document.getElementById('video1');
const video2 = document.getElementById('video2');
const video3 = document.getElementById('video3');

video2.style.display = 'none';
video3.style.display = 'none';

const slide = document.getElementsByClassName('slidecontainer')[0];
const slide2 = document.getElementsByClassName('slidecontainer')[1];
const slide3 = document.getElementsByClassName('slidecontainer')[2];

const enlarge = document.getElementById('vidEnlarge');

const videoSquare = document.getElementsByClassName('Video')[0];

launch.addEventListener('click', () => {
    launchPage.style.display = 'block';
    dataPage.style.display = 'none';
    data.classList.remove('current-heading');
    launch.classList.add('current-heading');
});

data.addEventListener('click', () => {
    launchPage.style.display = 'none';
    dataPage.style.display = 'block';
    launch.classList.remove('current-heading');
    data.classList.add('current-heading');
});

cam1.addEventListener('click', () => {
    video1.style.display = 'block';
    video2.style.display = 'none';
    video3.style.display = 'none';
    camMulti.classList.remove('current-heading');
    cam3.classList.remove('current-heading');
    cam2.classList.remove('current-heading');
    cam1.classList.add('current-heading');
    // video1.classList.remove('enlargeDualVideo');
    video2.classList.remove('enlargeDualVideo');
    video3.classList.remove('enlargeDualVideo');
    video2.classList.remove('dualVideo');
    video3.classList.remove('dualVideo');
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        slide.classList.remove('hideSlide');
        slide2.classList.add('hideSlide');
        slide3.classList.add('hideSlide');
    }
});

cam2.addEventListener('click', () => {
    video1.style.display = 'none';
    video2.style.display = 'block';
    video3.style.display = 'none';
    camMulti.classList.remove('current-heading');
    cam3.classList.remove('current-heading');
    cam1.classList.remove('current-heading');
    cam2.classList.add('current-heading');
    // video1.classList.remove('enlargeDualVideo');
    video2.classList.remove('enlargeDualVideo');
    video3.classList.remove('enlargeDualVideo');
    video2.classList.remove('dualVideo');
    video3.classList.remove('dualVideo');
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        slide.classList.add('hideSlide');
        slide2.classList.remove('hideSlide');
        slide3.classList.add('hideSlide');
    }
});

cam3.addEventListener('click', () => {
    video1.style.display = 'none';
    video2.style.display = 'none';
    video3.style.display = 'block';
    cam1.classList.remove('current-heading');
    cam2.classList.remove('current-heading');
    cam3.classList.add('current-heading');
    camMulti.classList.remove('current-heading');
    // video1.classList.add('enlargeDualVideo');
    video2.classList.remove('enlargeDualVideo');
    video3.classList.remove('enlargeDualVideo');
    video2.classList.remove('dualVideo');
    video3.classList.remove('dualVideo');
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        slide.classList.add('hideSlide');
        slide2.classList.add('hideSlide');
        slide3.classList.remove('hideSlide');
    }
    
});

camMulti.addEventListener('click', () => {
    video1.style.display = 'none';
    video2.style.display = 'block';
    video3.style.display = 'block';
    cam1.classList.remove('current-heading');
    cam2.classList.remove('current-heading');
    cam3.classList.remove('current-heading');
    camMulti.classList.add('current-heading');
    // video1.classList.add('enlargeDualVideo');
    video2.classList.add('dualVideo');
    video3.classList.add('dualVideo');
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        video2.classList.add('enlargeDualVideo');
        video3.classList.add('enlargeDualVideo');
    }
    if(videoSquare.classList.contains('enlargedVideoWrapper')) {
        slide.classList.add('hideSlide');
        slide2.classList.add('hideSlide');
        slide3.classList.add('hideSlide');
    }
});


enlarge.addEventListener('click', () => {
    videoSquare.classList.toggle('enlargedVideoWrapper');
    video1.classList.toggle('enlargedVideo');
    video2.classList.toggle('enlargedVideo');
    video3.classList.toggle('enlargedVideo');
    if(cam1.classList.contains('current-heading')) {
        slide.classList.toggle('hideSlide');
    } else if(cam2.classList.contains('current-heading')) {
        slide2.classList.toggle('hideSlide');
    } else if(cam3.classList.contains('current-heading')) {
        slide3.classList.toggle('hideSlide');
    } else if(camMulti.classList.contains('current-heading')) {
        video2.classList.toggle('enlargeDualVideo');
        video3.classList.toggle('enlargeDualVideo');
    }

    if(enlarge.innerHTML === '^') {
        enlarge.innerHTML = '-';
    } else {
        enlarge.innerHTML = '^';
    }
});

