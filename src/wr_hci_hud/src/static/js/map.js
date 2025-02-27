

var map = L.map('map').setView([38.3753855364, -110.8302205892], 12);

// Add OpenStreetMap tiles
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    
}).addTo(map);

var robotMarker = L.marker([38.3753855364, -110.8302205892], {
    icon: L.icon({
        iconUrl: "https://cdn-icons-png.flaticon.com/512/15872/15872849.png",
        iconSize: [40, 41],
        iconAnchor: [12, 41],
        popupAnchor: [1, -34]
    })
}).addTo(map);

