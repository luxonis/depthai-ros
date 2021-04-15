sudo wget -qO- http://docs.luxonis.com/_static/install_dependencies.sh | bash
cd /tmp
git clone --recursive https://github.com/luxonis/depthai-core.git --branch develop
mkdir build
cmake .. -DBUILD_SHARED_LIBS=ON
cd ..
cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build --target install

