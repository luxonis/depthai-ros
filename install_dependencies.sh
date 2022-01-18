set -e
sudo wget -qO- https://raw.githubusercontent.com/luxonis/depthai-docs-website/master/source/_static/install_dependencies.sh | bash 
cd /tmp
git clone --recursive https://github.com/luxonis/depthai-core.git --branch main
cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build depthai-core/build --target install
cd /tmp
rm -r depthai-core
