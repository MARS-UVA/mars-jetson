CATCH2_DIR=../../../../Catch2
BUILD_DIR=../build
DEBUG_DIR=../build/Debug
if [ $# -eq 0 ]; then
    cmake -S $CATCH2_DIR -B $CATCH2_DIR/build
    cmake --build $CATCH2_DIR/build
    sudo cmake --install $CATCH2_DIR/build
    if [ ! -d $BUILD_DIR ]; then
        mkdir $BUILD_DIR
        cd $BUILD_DIR
    fi
    if [ ! -f $DEBUG_DIR/test ]; then
        cd $BUILD_DIR
        cmake ..
        make
    fi
    ./test
elif [ $1 = "clean" ]; then
    echo "Cleaning build directory"
    rm -rf $BUILD_DIR
elif [ $1 = "run" ]; then 
    echo "Running tests"
    cd $BUILD_DIR
    ./test
else
    echo "Invalid argument"
fi