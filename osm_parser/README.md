# OSM Parser

## Overview

This project downloads a map file from OSM(Open Street Map) (e.g. /data/taipei.osm) and parses the
XML file into underneath graph structure (node and arc) as OsmNode class.
I calculates the "cost" of each edge (arc) according to road type and speed limit. And implement two basic route algorithm (Dijkstra and A star).

## OSM
Detailed specification for the .osm file is defined in [OSM Wiki](https://wiki.openstreetmap.org/wiki/Main_Page). Basica conecps are:
* [OSM Node](https://wiki.openstreetmap.org/wiki/Node): A point with latitude and longitude.
* [OSM Way](https://wiki.openstreetmap.org/wiki/Way): A line connecting nodes.
  * Tag: attribute of the way (e.g. road type, one way or not...)

## Algorithm
See parse_osm.py for more detail implementation on parsing and routing logic.

## Learning
* OSM XML files are very messy and contains lots of information unrelated to routing. A 10MB .osm file could contains 10k nodes but only 4k of them are road network (others could be park, building ...)
* OSM [download UI](https://www.openstreetmap.org/export) has many restrictions like max nodes per download make getting a larger road network very hard.
* I haven't found a good data format to stored the parsed road network, meaning that I will have to parse the road network from scratch everytime... which is every inefficient
* There are some really handy Python library:
  * geojson: Eazy converting graph into RFC7946 GeoJSON format (Easy visualization using online tools or github)
  * geopy: calculating distance between coordinates (lon, lat), and convert between unites (km, miles).
* It would be better to use existing tools like OSMnx to avoid hand crafting duplicate works.

### CPP
Xerces parser is **extremely** slow. It does not parse the XML line by line but try to parse the
whole XML before returning any node, which is not idea in OSM XML case. Also OSM xml file has
many unnecessary info (user related tags, building, recreation area ...) so we also need to
do a full Dijkstra to find the connected component (assuming it will be the road network) and
delete anything else.

