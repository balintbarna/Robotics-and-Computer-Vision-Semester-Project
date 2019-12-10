# Template matching mini program
This code can generate tons of templates for a 3D object, then match a camera image to those templates, and return the best match's pose.
## How to build and use
You need covis, vtk and opencv2 binaries and make sure that CMake can find them.
```
mkdir build
mkdir t
cd build
cmake ..
make -j2
cd ../t
./../build/create_templates_cad ../files/doggy.ply --no-depth --no-cloud -r 0.5,0.8,1.1,1.4 --lat-begin 45 --lat-end 80 --lon-begin 0 --lon-end 360 --fov 45.5 --bc 1,0,0 -t 1 -s 4 -v
cd ..
./build/linemod t/ files/scene.png -t 50 -p 3
```