mkdir build
cd build
cmake ..
make
./Noise <path/to/file.pcd> <noiselvl>

lower number gives bigger noise

example

./Noise ../../Scanner25D.pcd 100
