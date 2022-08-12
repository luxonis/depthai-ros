set -e

echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
cd /tmp
git clone --recursive https://github.com/luxonis/depthai-core.git --branch main
cmake -Hdepthai-core -Bdepthai-core/build -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build depthai-core/build --target install
cd /tmp
rm -r depthai-core
