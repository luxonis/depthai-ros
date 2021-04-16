pythn3 -m pip install opencv-python
git clone --recursive https://github.com/luxonis/depthai-core.git --branch develop
cd depthai-core
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
cd ..
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build --target install
