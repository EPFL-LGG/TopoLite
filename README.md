# TopoLite

## Mac

1. Install boost

2. Buildiing the project. In Topolite root folder:
```
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
```


## Windows

1. Install boost

2. Install Visual Studio 2019

3. Install Visual Studio Linux Development Tools

4. Open the CMakeLists.txt file in Visual Studio 2019.


## Ubuntu 18.04

1. Install boost

``` 
# If not already on apt
sudo add-apt-repository ppa:boost-latest/ppa
sudo apt-get update
# 
sudo apt install libboost-all-dev
```

2. Buildiing the project. In Topolite root folder:

```
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j8
```
Note: the integer following `make -j` is the number of threads supported by your CPU architecture. Replace it with your optimal value.
