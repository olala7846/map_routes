# Link executables with library. Will generate ./a.out file
# TODO(remove Xerrces XML).
main: main.o osm_parser.o road_network.o osm_xml_utils.o
	clang++ main.o osm_parser.o road_network.o osm_xml_utils.o

main.o: main.cpp
	clang++ -c main.cpp -I /opt/include/

osm_parser.o: osm_parser.cpp
	clang++ -c osm_parser.cpp -I /opt/include/ -std=c++17

# 3rd party library should be copied under /opt/include
# in order for g++ to link.
road_network.o: road_network.cpp
	clang++ -c road_network.cpp -I /opt/include/ -std=c++17

osm_xml_utils.o: osm_xml_utils.cpp
	clang++ -c osm_xml_utils.cpp -I /opt/include -std=c++17
