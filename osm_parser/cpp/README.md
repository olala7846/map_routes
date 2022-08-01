# Route Planning C++ files
`/route_planning` contains route planning related C++ files

## Makefile, see makefile for how to link libraries

### Clang, LLVM

* [Clang](https://clang.llvm.org/)
* [LLVM](https://www.llvm.org/)
Note: use clang++ so it links the C++ standard library.

### Xerces-C
Xerces-C is very slow and difficult to set up. Also, I don't need the full W3C compliant. So
I decided to go with RapidXML instead (faster and only .hpp file).

[Stackoverflow question](https://stackoverflow.com/questions/33559402/xerces-no-library-found-after-install)
```shell
./configure CFLAGS="-arch x86_64" CXXFLAGS="-arch x86_64"
./configure --prefix=/opt

sudo make (this builds the library)
sudo make install (this installs the library)
```

### RapidXML
http://rapidxml.sourceforge.net/


* g++ search path for "#include" headers  `-I /opt/include`

* g++ search path for library path  `-L /opt/lib -lxerces-c`




