rm -r build
mkdir build
cd build
cmake ..
msbuild RopeSim.sln 
./Debug/RopeSim
cd ..