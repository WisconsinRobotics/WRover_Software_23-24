// var videoSrc1 = 'https://test-streams.mux.dev/x36xhzz/x36xhzz.m3u8';
// var videoSrc2 = 'http://sample.vodobox.net/skate_phantom_flex_4k/skate_phantom_flex_4k.m3u8';
// // var videoSrc3 = 'https://test-streams.mux.dev/x36xhzz/x36xhzz.m3u8';
// var hls = new Hls();
// var hls2 = new Hls();
// // var hls3 = new Hls();

// hls.on(Hls.Events.MEDIA_ATTACHED, function () {
//   console.log('video and hls.js are now bound together !');
// });
// hls.on(Hls.Events.MANIFEST_PARSED, function (event, data) {
//   console.log(
//     'manifest loaded, found ' + data.levels.length + ' quality level',
//   );

//   console.log(data)
// });

// if (Hls.isSupported()) {
//     console.log('HLS supported')
    
//     hls.loadSource(videoSrc1);
//     hls.attachMedia(video1);
    
//     hls2.loadSource(videoSrc2);
//     hls2.attachMedia(video2);

//     // hls3.loadSource(videoSrc3);
//     // hls3.attachMedia(video3);
//     video1.play();
//     video2.play();
//     // video3.play();

// }

//zooming

var zoomBar = document.getElementById('myRange');
var zoomValue = zoomBar.getAttribute('value');
var zoomDiv1 = document.getElementById('zoomingDiv1');

var zoomBar2 = document.getElementById('myRange2');
var zoomValue2 = zoomBar2.getAttribute('value');
var zoomDiv2 = document.getElementById('zoomingDiv2');

var zoomBar3 = document.getElementById('myRange3');
var zoomValue3 = zoomBar3.getAttribute('value');
var zoomDiv3 = document.getElementById('zoomingDiv3');

var zoomBar4 = document.getElementById('myRange4');
var zoomValue4 = zoomBar4.getAttribute('value');
var zoomDiv4 = document.getElementById('zoomingDiv4');

zoomBar.oninput = function () {
    zoomValue = this.value;
    // video1.style.float = 'none';
    console.log(zoomValue);
    video1.style.transform = 'scale(' + zoomValue/10 + ')';
    // video.style.width = 1000 * zoomValue + 'px';
    // video.style.height = 500 * zoomValue + 'px';
}

zoomBar2.oninput = function () {
    zoomValue2 = this.value;
    video2.style.float = 'none';
    video2.style.transform = 'scale(' + zoomValue2/10 + ')';
}

//add vid3
zoomBar3.oninput = function () {
    zoomValue3 = this.value;
    video3.style.float = 'none';
    video3.style.transform = 'scale(' + zoomValue3/10 + ')';
}

zoomBar4.oninput = function () {
  zoomValue4 = this.value;
  video4.style.float = 'none';
  video4.style.transform = 'scale(' + zoomValue4/10 + ')';
}