
#!/bin/bash

BUILD_DIR="../build"
DEBUG_DIR="../build/Debug"

# Check if no argument passed or first-time setup
if [ $# -eq 0 ]; then
    echo "Starting fresh build..."
    
    if [ -d "$BUILD_DIR" ]; then
        echo "Cleaning existing build directory..."
        rm -rf "$BUILD_DIR"
    fi
    
    echo "Extracting RealSense SDK..."
    bash ./realsense_extract.sh
    
    echo "Creating build directory..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR" || exit
    
    echo "Running CMake..."
    cmake .. -G "Unix Makefiles" \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_WITH_CUDA=OFF \
        -DBUILD_NETWORK_DEVICE=OFF \
        -DBUILD_WITH_STATIC_CRT=OFF
    
    echo "Building RealSense library..."
    cmake --build . --config Debug --target realsense2
    
    echo "Building capture application..."
    cmake --build . --config Debug --target obstacle_detect_node
    cmake --build . --config Debug --target realsense_capture
    cmake --build . --config Debug --target gradientMapTest
    cmake --build . --config Debug --target obstacleDetectionTest
    # cmake --build . --config Debug --target obstacleClusteringClosestObstacleTest
    # cmake --build . --config Debug --target localPathPlanningTest
    # cmake --build . --config Debug --target pcTreeSimpleTests
    # cmake --build . --config Debug --target pcTreeQuadrantTest
    # cmake --build . --config Debug --target pcTreeGradientTest
    
    if [ -f "$DEBUG_DIR/realsense_capture" ]; then
        echo "Running application..."
        cd "$DEBUG_DIR" || exit
        ./realsense_capture 1
    else
        echo "Error: Build failed: executable not found" >&2
        exit 1
    fi
elif [ "$1" = "clean" ]; then
    echo "Cleaning build directory"
    rm -rf "$BUILD_DIR"
elif [ "$1" = "run" ]; then
    if [ -f "$DEBUG_DIR/realsense_capture" ]; then
        cd "$DEBUG_DIR" || exit
        ./realsense_capture 1
    else
        echo "Error: Executable not found. Please build first." >&2
        exit 1
    fi
else
    echo "Invalid argument"
    exit 1
fi