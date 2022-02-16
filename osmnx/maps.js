function initMap() {

  const center = { lat: 25.0338, lng: 121.5646 };
  const map = new google.maps.Map(document.getElementById("map"), {
    zoom: 13,
    center: center,
  });

  initMarkers(map);
}

function initMarkers(map) {

  // Taipei 101 25.0338° N, 121.5646° E
  const sourceLocation = { lat: 25.0338, lng: 121.5646 };
  // 師大夜市 25.023870 121.546800
  const targetLocation = { lat: 25.023879, lng: 121.546800 };

  let route = new google.maps.Polyline({
    map: map,
    strokeColor: 'blue',
  });

  // The marker, positioned at Taipei 101
  const sourceMarker = addMarker(sourceLocation, 'S', map);
  sourceMarker.addListener("dragend", (event)=> {
    sourceLocation.lat = event.latLng.lat();
    sourceLocation.lng = event.latLng.lng();
    drawLine(route, sourceLocation, targetLocation);
  });

  const targetMarker = addMarker(targetLocation, 'T', map);
  targetMarker.addListener("dragend", (event) =>{
    console.log('target dragend', event.latLng.lat(), event.latLng.lng());
    targetLocation.lat = event.latLng.lat();
    targetLocation.lng = event.latLng.lng();
    drawLine(route, sourceLocation, targetLocation);
  });

  drawLine(route, sourceLocation, targetLocation);
}

function drawLine(polyline, sourceLocation, destLocation) {
  // polyline.setPath([sourceLocation, destLocation]);
  drawLineServer(polyline, sourceLocation, destLocation);
}

function drawLineServer(polyline, sourceLocation, destLocation) {
  let request_path = `http://localhost:8080/route?orig=${sourceLocation.lat},${sourceLocation.lng}&dest=${destLocation.lat},${destLocation.lng}`;
  let request = new Request(request_path);
  fetch(request)
    .then((response) => response.json())
    .then((json) => {
      path = json.route.map(function (point) {
        return {
          lat: point[0],
          lng: point[1]
        };
      });
      polyline.setPath(path);
    });
  // polyline.setPath([sourceLocation, destLocation]);
}

function addMarker(location, label, map) {
  let marker = new google.maps.Marker({
    position: location,
    label: label,
    map: map,
    draggable: true,
  });
  return marker;
}

function updateLatLng(location, event) {
  location.lat = event.latLng.lat();
  location.lng = event.latLng.lng();
}
