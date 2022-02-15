let map;

function initMap() {

  // Taipei City 25.0330° N, 121.5654° E
  // map = new google.maps.Map(document.getElementById("map"), {
  //   center: { lat: 25.0330, lng: 121.5654 },
  //   zoom: 13,
  // });

  // Taipei 101 25.0338° N, 121.5646° E
  const tp101 = { lat: 25.0338, lng: 121.5646 };
  // The map, centered at Uluru
  const map = new google.maps.Map(document.getElementById("map"), {
    zoom: 13,
    center: tp101,
  });
  // The marker, positioned at Uluru
  const marker = new google.maps.Marker({
    position: tp101,
    map: map,
  });
}
