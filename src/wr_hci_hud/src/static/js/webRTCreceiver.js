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

function initRemoteStreams(api) {
  const remoteStreamsElement = document.getElementById("zoomingDiv4");

  const listener = {
    producerAdded: function(producer) {
      const producerId = producer.id
      if (!document.getElementById(producerId)) {
        remoteStreamsElement.insertAdjacentHTML("beforeend",
          `<li id="${producerId}">
            <div class="button">${producer.meta.name || producerId}
            </div>
            <div class="offer-options">
              <textarea rows="5" cols="50" placeholder="offer options, empty to answer. For example:\n{\n  &quot;offerToReceiveAudio&quot;: 1\n  &quot;offerToReceiveVideo&quot;: 1\n}\n"></textarea>
            </div>
            <div class="request-box">
              <textarea rows="4" cols="50" placeholder="JSON request to send over"></textarea>
              <button disabled="disabled">Submit request</button>
            </div>
            <div class="video">
                <div class="spinner">
                    <div></div>
                    <div></div>
                    <div></div>
                    <div></div>
                </div>
                <span class="remote-control">&#xA9;</span>
                <video></video>
                <div class="fullscreen"><span title="Toggle fullscreen">&#x25A2;</span></div>
            </div>
          </li>`);

        const entryElement = document.getElementById(producerId);
        const videoElement = entryElement.getElementsByTagName("video")[0];
        const offerTextareaElement = entryElement.getElementsByTagName("textarea")[0];
        const requestTextAreaElement = entryElement.getElementsByTagName("textarea")[1];
        const submitRequestButtonElement = entryElement.getElementsByTagName("button")[0];

        submitRequestButtonElement.addEventListener("click", (event) => {
          try {
            let request = requestTextAreaElement.value;
            let id = entryElement._consumerSession.remoteController.sendControlRequest(request);
          } catch (ex) {
            console.error("Failed to parse mix matrix:", ex);
            return;
          }
        });

        videoElement.addEventListener("playing", () => {
          if (entryElement.classList.contains("has-session")) {
            entryElement.classList.add("streaming");
          }
        });

        entryElement.addEventListener("click", (event) => {
          event.preventDefault();
          if (!event.target.classList.contains("button")) {
            return;
          }

          if (entryElement._consumerSession) {
            entryElement._consumerSession.close();
          } else {
            let session = null;
            if (offerTextareaElement.value == '') {
              session = api.createConsumerSession(producerId);
            } else {
              try {
                let offerOptions = JSON.parse(offerTextareaElement.value);
                session = api.createConsumerSessionWithOfferOptions(producerId, offerOptions);
              } catch (ex) {
                console.error("Failed to parse offer options:", ex);
                return;
              }
            }
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
                  videoElement.pause();
                  videoElement.srcObject = null;
                  entryElement.classList.remove("has-session", "streaming", "has-remote-control");
                  delete entryElement._consumerSession;
                }
              });

              session.addEventListener("streamsChanged", () => {
                if (entryElement._consumerSession === session) {
                  const streams = session.streams;
                  if (streams.length > 0) {
                    videoElement.srcObject = streams[0];
                    videoElement.play().catch(() => {});
                  }
                }
              });

              session.addEventListener("remoteControllerChanged", () => {
                if (entryElement._consumerSession === session) {
                  const remoteController = session.remoteController;
                  if (remoteController) {
                    entryElement.classList.add("has-remote-control");
                    submitRequestButtonElement.disabled = false;
                    remoteController.attachVideoElement(videoElement);
                    remoteController.addEventListener("info", (e) => {
                      console.log("Received info message from producer: ", e.detail);
                    });
                  } else {
                    entryElement.classList.remove("has-remote-control");
                    submitRequestButtonElement.disabled = true;
                  }
                }
              });

              entryElement.classList.add("has-session");
              session.connect();
            }
          }
        });
      }
    },

    producerRemoved: function(producer) {
      const element = document.getElementById(producer.id);
      if (element) {
        if (element._consumerSession) {
          element._consumerSession.close();
        }

        element.remove();
      }
    }
  };

  api.registerProducersListener(listener);
  for (const producer of api.getAvailableProducers()) {
    listener.producerAdded(producer);
  }
}

window.addEventListener("DOMContentLoaded", () => {
  document.addEventListener("click", (event) => {
    if (event.target.matches("div.video>div.fullscreen:hover>span")) {
      event.preventDefault();
      event.target.parentNode.previousElementSibling.requestFullscreen();
    }
  });

  const signalingProtocol = window.location.protocol.startsWith("https") ? "wss" : "ws";
  const gstWebRTCConfig = {
    meta: { name: `WebClient-${Date.now()}` },
    signalingServerUrl: `${signalingProtocol}://${window.location.hostname}:8443`,
  };

  const api = new GstWebRTCAPI(gstWebRTCConfig);
  // initCapture(api);
  initRemoteStreams(api);
});