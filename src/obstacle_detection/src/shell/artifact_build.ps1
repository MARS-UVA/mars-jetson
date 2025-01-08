$BUILD_DIR = "..\build"
$DEBUG_DIR = "..\build\Debug"

# Check if no argument passed or first-time setup
if ($args.Count -eq 0) {
    if (-Not (Test-Path -Path $BUILD_DIR)) {
        New-Item -ItemType Directory -Path $BUILD_DIR
        Set-Location $BUILD_DIR
    }
    if (-Not (Test-Path -Path "$DEBUG_DIR\realsense_capture.exe")) {
        Set-Location $BUILD_DIR
        cmake .. -G "Visual Studio 17 2022" -A x64 -DBUILD_EXAMPLES=OFF -DBUILD_WITH_CUDA=OFF -DBUILD_NETWORK_DEVICE=OFF
        cmake --build . --config Debug --target realsense2
        cmake --build . --config Debug --target realsense_capture
    }
    Set-Location $DEBUG_DIR
    .\realsense_capture.exe
}
elseif ($args[0] -eq "clean") {
    Write-Host "Cleaning build directory"
    Remove-Item -Recurse -Force $BUILD_DIR
}
elseif ($args[0] -eq "run") {
    Write-Host "Running tests"
    .\realsense_capture.exe
}
else {
    Write-Host "Invalid argument"
}