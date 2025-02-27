function initCapture(api) {
  const captureSection = document.getElementById("capture");
  const clientIdElement = captureSection.querySelector(".client-id");
  const videoElement = captureSection.getElementsByTagName("video")[0];

  const listener = {
    connected: function(clientId) { clientIdElement.textContent = clientId; },
    disconnected: function() { clientIdElement.textContent = "none"; }
  };
  api.registerConnectionListener(listener);

  document.getElementById("capture-button").addEventListener("click", (event) => {
    event.preventDefault();

    if (captureSection._producerSession) {
      captureSection._producerSession.close();
    } else if (!captureSection.classList.contains("starting")) {
      captureSection.classList.add("starting");

      const constraints = {
        video: { width: 1280, height: 720 }
      };
      navigator.mediaDevices.getUserMedia(constraints).then((stream) => {
        const session = api.createProducerSession(stream);
        if (session) {
          captureSection._producerSession = session;

          session.addEventListener("error", (event) => {
            if (captureSection._producerSession === session) {
              console.error(event.message, event.error);
            }
          });

          session.addEventListener("closed", () => {
            if (captureSection._producerSession === session) {
              videoElement.pause();
              videoElement.srcObject = null;
              captureSection.classList.remove("has-session", "starting");
              delete captureSection._producerSession;
            }
          });

          session.addEventListener("stateChanged", (event) => {
            if ((captureSection._producerSession === session) &&
              (event.target.state === GstWebRTCAPI.SessionState.streaming)) {
              videoElement.srcObject = stream;
              videoElement.play().catch(() => {});
              captureSection.classList.remove("starting");
            }
          });

          session.addEventListener("clientConsumerAdded", (event) => {
            if (captureSection._producerSession === session) {
              console.info(`client consumer added: ${event.detail.peerId}`);
            }
          });

          session.addEventListener("clientConsumerRemoved", (event) => {
            if (captureSection._producerSession === session) {
              console.info(`client consumer removed: ${event.detail.peerId}`);
            }
          });

          captureSection.classList.add("has-session");
          session.start();
        } else {
          for (const track of stream.getTracks()) {
            track.stop();
          }

          captureSection.classList.remove("starting");
        }
      }).catch((error) => {
        console.error("cannot have access to webcam and microphone", error);
        captureSection.classList.remove("starting");
      });
    }
  });
}

function producerAddedHandler(api, producer, id) {
  const producerId = producer.id

  const entryElement = document.getElementById(id);
  // const videoElement = entryElement.getElementsByTagName("video")[0];
  // const videoElement = entryElement;
  entryElement.addEventListener("playing", () => {
    if (entryElement.classList.contains("has-session")) {
      entryElement.classList.add("streaming");
    }
  });

  // entryElement.addEventListener("click", (event) => {
  //   event.preventDefault();

    if (entryElement._consumerSession) {
      entryElement._consumerSession.close();
    } else {
      let session = null;
      session = api.createConsumerSession(producerId);
      
      if (session) {
        entryElement._consumerSession = session;

        session.mungeStereoHack = true;

        session.addEventListener("error", (event) => {
          if (entryElement._consumerSession === session) {
            console.error(event.message, event.error);
          }
        });

        session.addEventListener("closed", () => {
          if (entryElement._consumerSession === session) {
            entryElement.pause();
            entryElement.srcObject = null;
            entryElement.classList.remove("has-session", "streaming", "has-remote-control");
            delete entryElement._consumerSession;
          }
        });

        session.addEventListener("streamsChanged", () => {
          if (entryElement._consumerSession === session) {
            const streams = session.streams;
            if (streams.length > 0) {
              entryElement.srcObject = streams[0];
              entryElement.play().catch(() => {});
            }
          }
        });

        // session.addEventListener("remoteControllerChanged", () => {

        entryElement.classList.add("has-session");
        session.connect();
      }
    }
  // });
}

let counter = 0;
function initRemoteStreams(api) {
  // const remoteStreamsElement = document.getElementById("zoomingDiv4");

  const listener = {
    producerAdded: function(producer) {
      if( counter == 0){
        producerAddedHandler(api, producer, "video1");
      }
      else if( counter == 1){
        producerAddedHandler(api, producer, "video2");
      }
      else if( counter == 2){
        producerAddedHandler(api, producer, "video3");
      }
      else if( counter == 3){
        producerAddedHandler(api, producer, "video4");
      }
      else{
        console.log("No more divs available");
      }
      counter++;
    },

    producerRemoved: function(producer) {
      // const element = document.getElementById(producer.id);
      // if (element) {
      //   if (element._consumerSession) {
      //     element._consumerSession.close();
      //   }

      //   element.remove();
      // }
    }
  };

  api.registerProducersListener(listener);
  for (const producer of api.getAvailableProducers()) {
    listener.producerAdded(producer);
  }
}

window.addEventListener("DOMContentLoaded", async () => {
  document.addEventListener("click", (event) => {
    if (event.target.matches("div.video>div.fullscreen:hover>span")) {
      event.preventDefault();
      event.target.parentNode.previousElementSibling.requestFullscreen();
    }
  });

  const signalingProtocol = window.location.protocol.startsWith("https") ? "wss" : "ws";
  const gstWebRTCConfig = {
    meta: { name: `WebClient-${Date.now()}` },
    signalingServerUrl: `${signalingProtocol}://${await getHostIP()}:8443`,
  };

  const api = new GstWebRTCAPI(gstWebRTCConfig);
  // initCapture(api);
  initRemoteStreams(api);
});