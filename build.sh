#!/bin/bash
Help()
{
    echo "Build workspace"
    echo
    echo "Build options:"
    echo "-s [1]   Set to 1 to build sequentially (longer, but saves RAM & CPU)"
    echo "-r [0]  Set to 1 to build in Debug mode. (RelWithDebInfo)"
    echo "-m [0]   Set to 1 to build with --merge-install option."
    echo
}

sequential=1
release=0
merge=0
build_type=Release
install_type=symlink-install
while getopts ":h:s:r:m:" option; do
   case $option in
      h) # display Help
         Help
         exit;;
      s) # Sequential executor
         sequential=$OPTARG;;
      r) # Build type
         release=$OPTARG;;
      m) # Install type
         merge=$OPTARG;;
     \?) # Invalid option
         echo "Error: Invalid option"
         exit;;
   esac
done

if [ "$release" == 0 ]
then
    build_type="RelWithDebInfo"
fi

if [ "$merge" == 1 ]
then
    install_type="merge-install"
fi
echo "Build type: $build_type, Install_type: $install_type"
if [ "$sequential" == 1 ]
then
    echo "Sequential build" && \
    MAKEFLAGS="-j1 -l1" colcon build \
        --$install_type \
        --executor sequential \
        --cmake-args -DCMAKE_BUILD_TYPE=$build_type \
        --cmake-args -DBUILD_TESTING=OFF \
        --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
        --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
        --cmake-args -DBUILD_SHARED_LIBS=ON
else
    echo "Parallel build" && \
    colcon build \
    --$install_type \
    --cmake-args -DCMAKE_BUILD_TYPE=$build_type \
    --cmake-args -DBUILD_TESTING=OFF --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON --cmake-args -DCMAKE_POSITION_INDEPENDENT_CODE=ON --cmake-args -DBUILD_SHARED_LIBS=ON
fi