# Template matching mini program
This code can generate tons of templates for a 3D object, then match a camera image to those templates, and return the best match's pose.
## How to build and use
You need covis, vtk and opencv2 binaries and make sure that CMake can find them.
```
mkdir build
mkdir templates
cd build
cmake ..
make -j
cd ../templates
./../build/create_templates_cad ../files/obj_09.ply --no-depth --no-cloud -r 650,700,750
./linemod ../templates/ ../files/1017.png -t 50 -p 3
```