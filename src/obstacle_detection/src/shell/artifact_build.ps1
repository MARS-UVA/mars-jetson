$BUILD_DIR = "..\build"
$DEBUG_DIR = "..\build\Debug"

# Check if no argument passed or first-time setup
if ($args.Count -eq 0) {
    Write-Host "Starting fresh build..."
    
    # Clean build directory if it exists
    # if (Test-Path -Path $BUILD_DIR) {
    #     Write-Host "Cleaning existing build directory..."
    #     Remove-Item -Recurse -Force $BUILD_DIR
    # }
    
    Write-Host "Extracting RealSense SDK..."
    . ".\realsense_extract.ps1"
    
    Write-Host "Creating build directory..."
    New-Item -ItemType Directory -Path $BUILD_DIR
    Set-Location $BUILD_DIR
    
    Write-Host "Running CMake..."
    cmake .. -G "Visual Studio 17 2022" -A x64 `
        -DBUILD_EXAMPLES=OFF `
        -DBUILD_WITH_CUDA=OFF `
        -DBUILD_NETWORK_DEVICE=OFF `
        -DBUILD_WITH_STATIC_CRT=OFF
    
    Write-Host "Building RealSense library..."
    cmake --build . --config Debug --target realsense2
    
    Write-Host "Building capture application..."
    cmake --build . --config Debug --target realsense_capture
    cmake --build . --config Debug --target gradientMapTest
    cmake --build . --config Debug --target obstacleDetectionTest
    # cmake --build . --config Debug --target obstacleClusteringClosestObstacleTest
    # cmake --build . --config Debug --target localPathPlanningTest
    # cmake --build . --config Debug --target pcTreeSimpleTests
    # cmake --build . --config Debug --target pcTreeQuadrantTest
    # cmake --build . --config Debug --target pcTreeGradientTest
    
    if (Test-Path -Path "$DEBUG_DIR\realsense_capture.exe") {
        Write-Host "Running application..."
        Set-Location $DEBUG_DIR
        .\realsense_capture.exe 1
    }
    else {
        Write-Error "Build failed: executable not found"
    }
}
elseif ($args[0] -eq "clean") {
    Write-Host "Cleaning build directory"
    Remove-Item -Recurse -Force $BUILD_DIR
}
elseif ($args[0] -eq "run") {
    if (Test-Path -Path "$DEBUG_DIR\realsense_capture.exe") {
        Set-Location $DEBUG_DIR
        .\realsense_capture.exe 1
    }
    else {
        Write-Error "Executable not found. Please build first."
    }
}
else {
    Write-Host "Invalid argument"
}