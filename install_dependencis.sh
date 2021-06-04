sudo wget -qO- http://docs.luxonis.com/_static/install_dependencies.sh | bash
cd /tmp
git clone --recursive https://github.com/luxonis/depthai-core.git --branch ros2-main-gen2
cd depthai-core
mkdir build
cmake -S . -B build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build --target install
cd /tmp
rm -r depthai-core
