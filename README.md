# shinji
Global Localization



### Build from source

```
# build teaser++ library
cd thirdparty/teaserpp
mkdir build && cd build
cmake .. && make -j 12

# build shinji ros package
catkin build shinji -DCMAKE_BUILD_TYPE=Release
```



### Pointcloud map preprocess

```
cd thirdparty/preprocess
mkdir build && cd build
cmake .. && make -j 12

./preprocess ../config.yaml
```



### usage

```
rosservice call /shinji/query
```

```
rostopic echo /shinji/result
```

