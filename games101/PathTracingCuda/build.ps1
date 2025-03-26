rm -r build
mkdir build
cd build
cmake ..
msbuild PathTracingCuda.sln 
./Debug/PathTracingCuda
cd ..