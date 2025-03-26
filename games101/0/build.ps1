rm -r build
mkdir build
cd build
cmake ..
msbuild Transformation.sln
./Debug/Transformation
cd ..