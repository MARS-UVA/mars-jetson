$BUILD_DIR = "build"
$DEBUG_DIR = "build\Debug"

# Check if no argument passed or first-time setup
if ($args.Count -eq 0) {
    if (-Not (Test-Path -Path $BUILD_DIR)) {
        New-Item -ItemType Directory -Path $BUILD_DIR
        Set-Location $BUILD_DIR
    }
    if (-Not (Test-Path -Path $DEBUG_DIR + "\test.exe")) {
        Set-Location $BUILD_DIR
        cmake .. -G "Visual Studio 16 2019"
        cmake --build .
    }
    Set-Location $DEBUG_DIR
    .\test.exe
}
elseif ($args[0] -eq "clean") {
    Write-Host "Cleaning build directory"
    Remove-Item -Recurse -Force $BUILD_DIR
}
elseif ($args[0] -eq "run") {
    Write-Host "Running tests"
    .\test.exe
}
else {
    Write-Host "Invalid argument"
}
