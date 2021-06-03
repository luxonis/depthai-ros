sudo wget -qO- http://docs.luxonis.com/_static/install_dependencies.sh | bash
cd /tmp
git clone --recursive https://github.com/luxonis/depthai-core.git --branch develop
cd depthai-core
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
cd ..
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/usr/local
sudo cmake --build build --target install
cd /tmp
rm -r depthai-core
