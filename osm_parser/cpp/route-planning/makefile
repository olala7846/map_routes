# Link executables with library. Will generate ./a.out file
# TODO(remove Xerrces XML).
main: main.o road_network.o
	clang++ main.o road_network.o -o main.out

main.o: main.cpp
	clang++ -c main.cpp -I /opt/include/ -std=c++17

# 3rd party library should be copied under /opt/include
# in order for g++ to link.
road_network.o: road_network.cpp
	clang++ -c road_network.cpp -I /opt/include/ -std=c++17
