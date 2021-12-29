# Route Planning C++ files
`/route_planning` contains route planning related C++ files

## Makefile, see makefile for how to link libraries

### Xerces-C
[Stackoverflow question](https://stackoverflow.com/questions/33559402/xerces-no-library-found-after-install)
```shell
./configure CFLAGS="-arch x86_64" CXXFLAGS="-arch x86_64"
./configure --prefix=/opt

sudo make (this builds the library)
sudo make install (this installs the library)
```

* g++ search path for "#include" headers  `-I /opt/include`

* g++ search path for library path  `-L /opt/lib -lxerces-c`



