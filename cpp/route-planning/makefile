main: main.o osm_parser.o
	g++ main.o osm_parser.o -L /opt/lib -lxerces-c

main.o: main.cpp
	g++ -c main.cpp -I /opt/include/

osm_parser.o: osm_parser.cpp
	g++ -c osm_parser.cpp -I /opt/include/ -std=c++11
