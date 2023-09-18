if [ -d "build_release" ]; then
rm -rf build_release
fi

mkdir build_release
cd build_release
cmake .. -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_BUILD_TYPE=Release -DCLOUD_PLATFORM=on -G "Unix Makefiles"
make -j10

cd ../../bin/Release

if [ -f "AEB.zip" ]; then
rm -rf AEB.zip
fi

if [ -f "LKA.zip" ]; then
rm -rf LKA.zip
fi

zip -q AEB.zip libHDMapModule.so libSimOneAPI.so libSSD.so AEB
zip -q LKA.zip libHDMapModule.so libSimOneAPI.so libSSD.so LKA

cp -f AEB.zip LKA.zip ../../../../../WebApp/WebServers/AssetServer/assets/controller/linux