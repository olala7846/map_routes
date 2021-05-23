/// <reference types="@types/google.maps" />

import { Component, OnInit } from '@angular/core';
import { Loader } from "@googlemaps/js-api-loader";
import {} from 'google.maps';
import { MapType } from '@angular/compiler';
import { GOOGLE_MAPS_API_KEY } from './secrets';

@Component({
  selector: 'app-map',
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.css']
})
export class MapComponent implements OnInit {
  loader: Loader;
  map: google.maps.Map;
  mapOptions: google.maps.MapOptions;

  constructor() {
    this.loader = new Loader({
      apiKey: GOOGLE_MAPS_API_KEY,
      version: "weekly",
      // language: 'zh-TW',
    });
    this.mapOptions = {
      center: {
        lat: 25.0480331,
        lng: 121.5329048,
      },
      zoom: 12,
      maxZoom: 20,
      mapTypeId: 'roadmap',   // Default 'roadmap'
      streetViewControl: false,
    }
  }

  ngOnInit(): void {
    this.loader.load().then(() => {
      let mapDiv = document.getElementById('map');
      this.map = new google.maps.Map(mapDiv, this.mapOptions);
    });
  }

  onLoadGeoJson(): void {
    const fileName = '/assets/roads.geojson';
    this.map.data.loadGeoJson(fileName);
    console.log('load ' + fileName);
  }
}
