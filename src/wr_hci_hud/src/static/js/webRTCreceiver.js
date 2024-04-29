var hostArray = window.location.host.split(':');
console.log(hostArray);
  var serverLoc = 'ws://' + hostArray[0] + ':8080/'
  var socket = new WebSocket(serverLoc); 

  var localvid = document.getElementById('localvideo');
  var remotevid = document.getElementById('remotevideo');
  var localStream = null;
  var pc = null;
  var mediaFlowing = false;
  var useH264 = true;
  var mediaConstraints = {'mandatory': {
                          'offerToReceiveAudio':true, 
                          'offerToReceiveVideo':true }};


  function useH264Codec(sdp) {

    var isFirefox = typeof InstallTrigger !== 'undefined';
    if (isFirefox)
        updated_sdp = sdp.replace("m=video 9 UDP/TLS/RTP/SAVPF 120 126 97\r\n","m=video 9 UDP/TLS/RTP/SAVPF 126 120 97\r\n");
    else
        updated_sdp = sdp.replace("m=video 9 UDP/TLS/RTP/SAVPF 100 101 107 116 117 96 97 99 98\r\n","m=video 9 UDP/TLS/RTP/SAVPF 107 101 100 116 117 96 97 99 98\r\n");

    return updated_sdp;
  }

  function setLocalDescAndSendMessageOffer(sessionDescription) {

    if (useH264) {
      // use H264 video codec in offer every time
      sessionDescription.sdp = useH264Codec(sessionDescription.sdp); 
    }

    pc.setLocalDescription(sessionDescription);

    console.log("Sending SDP offer: ");
    console.log(sessionDescription);

    socket.send(JSON.stringify({
                  "messageType": "offer",
                  "peerDescription": sessionDescription
             }));
  }

  function setLocalDescAndSendMessageAnswer(sessionDescription) {

    if (useH264) {
      // use H264 video codec in offer every time 
      sessionDescription.sdp = useH264Codec(sessionDescription.sdp);
    }
    pc.setLocalDescription(sessionDescription);
       
    console.log("Sending SDP answer:");
    console.log(sessionDescription);

    socket.send(JSON.stringify({
                  "messageType": "answer",
                  "peerDescription": sessionDescription
             }));
  }

  function onCreateOfferFailed() {
    console.log("Create Offer failed");
  }

  // start the connection on button click 
  function connect() {
    if (!mediaFlowing && localStream) {
      createPeerConnection();
      mediaFlowing = true;
      pc.createOffer(setLocalDescAndSendMessageOffer, onCreateOfferFailed, mediaConstraints);
    } else {
      alert("Local stream not running yet or media still flowing");
    }
  }

  // stop the connection on button click 
  function disconnect() {
    console.log("disconnect.");    
    socket.send(JSON.stringify({messageType: "bye"}));
    stop();
  }

  function stop() {
    pc.close();
    pc = null;
    remotevid.src = null; 
    mediaFlowing = false;    
  }

  function onCreateAnswerFailed(error) {
    console.log("Create Answer failed:",error);
  }

  socket.addEventListener("message", onWebSocketMessage, false);

  // process messages from web socket 
  function onWebSocketMessage(evt) {
    var message = JSON.parse(evt.data);

    if (message.messageType === 'offer') {
      console.log("Received offer...")
      console.log(evt);
      if (!mediaFlowing) {
        createPeerConnection();
        mediaFlowing = true;
      }
      console.log('Creating remote session description...' );

      var remoteDescription = message.peerDescription;
      
      var RTCSessionDescription = window.RTCSessionDescription || window.webkitRTCSessionDescription || window.RTCSessionDescription;
      pc.setRemoteDescription(new RTCSessionDescription(remoteDescription), function() {
        console.log('Sending answer...');
        pc.createAnswer(setLocalDescAndSendMessageAnswer, onCreateAnswerFailed);
      }, function() {  
        console.log('Error setting remote description');   
      });

    } else if (message.messageType === 'answer' && mediaFlowing) {
      console.log('Received answer...');
      console.log('Setting remote session description...' );
      var remoteDescription = message.peerDescription;
      var RTCSessionDescription = window.RTCSessionDescription || window.webkitRTCSessionDescription || window.RTCSessionDescription;
      pc.setRemoteDescription(new RTCSessionDescription(remoteDescription));

    } else if (message.messageType === "iceCandidate" && mediaFlowing) {
      console.log('Received ICE candidate...');
      var RTCIceCandidate = window.RTCIceCandidate || window.webkitRTCIceCandidate || window.RTCIceCandidate;
      var candidate = new RTCIceCandidate({sdpMLineIndex:message.candidate.sdpMLineIndex, sdpMid:message.candidate.sdpMid, candidate:message.candidate.candidate});
      pc.addIceCandidate(candidate );

    } else if (message.messageType === 'bye' && mediaFlowing) {
      console.log("Received bye");
      stop();
    }
  }

  function createPeerConnection() {
    console.log("Creating peer connection");
    RTCPeerConnection = window.webkitRTCPeerConnection || window.RTCPeerConnection;
    var pc_config = {"iceServers":[]};
    try {
      pc = new RTCPeerConnection(pc_config);
    } catch (e) {
      console.log("Failed to create PeerConnection, exception: " + e.message);
    }
    // send any ice candidates to the other peer
    pc.onicecandidate = function (evt) {
      if (evt.candidate) {
        console.log('Sending ICE candidate...');
        console.log(evt.candidate);

        socket.send(JSON.stringify({
                     "messageType": "iceCandidate",
                     "candidate": evt.candidate 
                    }));   
      } else {
        console.log("End of candidates.");
      }
    };
    console.log('Adding local stream...');
    pc.addStream(localStream);

    pc.addEventListener("addstream", onRemoteStreamAdded, false);
    pc.addEventListener("removestream", onRemoteStreamRemoved, false)

    // when remote adds a stream, hand it on to the local video element
    function onRemoteStreamAdded(evt) {
      console.log("Added remote stream");
      remotevid.src = window.URL.createObjectURL(evt.stream);
    }

    // when remote removes a stream, remove it from the local video element
    function onRemoteStreamRemoved(evt) {
      console.log("Remove remote stream");
      remotevid.src = "";
    }
  }