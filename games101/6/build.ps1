rm -r build
mkdir build
cd build
cmake ..
msbuild RayTracing.sln 
./Debug/RayTracing
cd ..