cd ORB_SLAM2/Thirdparty/DBoW2
echo "Configuring and building Thirdparty/DBoW2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../
echo "Configuring and building Pangolin ..."

git clone https://github.com/stevenlovegrove/Pangolin
cd Pangolin
mkdir build
cd build
cmake .. 
make -j4

cd ../../../
echo "Configuring and building ORB_SLAM2 ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
