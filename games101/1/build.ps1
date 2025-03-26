rm -r build
mkdir build
cd build
cmake ..
msbuild Rasterizer.sln 
./Debug/Rasterizer
cd ..