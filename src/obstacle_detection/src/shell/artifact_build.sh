BUILD_DIR=../build
DEBUG_DIR=../build/Debug
sudo apt-get install xorg-dev libglu1-mesa-dev
sudo apt install libssl-dev
if [ $# == 0 ]; then
    if [ ! -d $BUILD_DIR ]; then
        mkdir $BUILD_DIR
        cd $BUILD_DIR
    fi
    if [ ! -f $DEBUG_DIR/realsense_capture ]; then
        cd $BUILD_DIR
        rm $BUILD_DIR ## austen was here hehehehoohohoohoho
        cmake ..
        make
    fi
    ./realsense_capture
elif [ $1 == "clean" ]; then
    echo "Cleaning build directory"
    rm -rf $BUILD_DIR
elif [ $1 == "run" ]; then 
    echo "Running tests"
    cd $BUILD_DIR
    ./realsense_capture
else
    echo "Invalid argument"
fi

