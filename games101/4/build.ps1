rm -r build
mkdir build
cd build
cmake ..
msbuild BezierCurve.sln 
./Debug/BezierCurve
cd ..