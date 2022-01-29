main: main.o osm_parser.o road_network.o osm_xml_utils.o
	g++ main.o osm_parser.o road_network.o osm_xml_utils.o -L /opt/lib -lxerces-c

main.o: main.cpp
	g++ -c main.cpp -I /opt/include/

osm_parser.o: osm_parser.cpp
	g++ -c osm_parser.cpp -I /opt/include/ -std=c++11

road_network.o: road_network.cpp
	g++ -c road_network.cpp -I /opt/include/ -std=c++11

osm_xml_utils.o: osm_xml_utils.cpp
	g++ -c osm_xml_utils.cpp -I /opt/include -std=c++11
