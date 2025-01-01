BUILD_DIR=build
DEBUG_DIR=build/Debug
if [ $# == 0 ]; then
    if [ ! -d $BUILD_DIR ]; then
        mkdir $BUILD_DIR
        cd $BUILD_DIR
    fi
    if [ ! -f $DEBUG_DIR/test ]; then
        cd $BUILD_DIR
        cmake ..
        make
    fi
    cd $DEBUG_DIR
    ./test
elif [ $1 == "clean" ]; then
    echo "Cleaning build directory"
    rm -rf $BUILD_DIR
elif [ $1 == "run" ]; then 
    echo "Running tests"
    ./test
else
    echo "Invalid argument"
fi