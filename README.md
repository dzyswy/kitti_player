# kitti_player
kitti dataset ros player





## 编译

### 依赖
```
ros
pcl
opencv
boost
protobuf
glog
```


### 编译和运行
```
mkdir build
cd build
cmake ..
make -j4
./bin/player_demo ../cfg/prototxt/velodyne_only.prototxt


```













