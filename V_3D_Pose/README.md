# 3D pose estimation
This code can read a scene file and an object file. It will process both pointclouds and do global and local alignment on the object. It will return the alignment transformation.

You need pcl library to compile

```
mkdir build
cd build
cmake ..
make
./poseEstimation ../Scanner25D.pcd
```