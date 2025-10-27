$sourcePath = "librealsense-2.56.3"
$sdkUrl = "https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.56.3.zip"

$downloadDir = "./realsense-source"
$zipPath = Join-Path $downloadDir "realsense-sdk-v2.56.3.zip"
$completeDir = Join-Path $downloadDir $sourcePath
try {
    if (-Not (Test-Path -Path $completeDir)) {
        try {
            if (-not (Test-Path $downloadDir)) {
                New-Item -ItemType Directory -Path $downloadDir -Force
                Write-Host "Created directory: $downloadDir"
            }

            [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

            Write-Host "Downloading Intel RealSense SDK v2.56.3..."
            
            $ProgressPreference = 'SilentlyContinue'
            Invoke-WebRequest -Uri $sdkUrl -OutFile $zipPath -UseBasicParsing
            
            Write-Host "Download completed successfully!"

            if (Test-Path $zipPath) {
                $fileSize = (Get-Item $zipPath).Length
                if ($fileSize -gt 0) {
                    Write-Host "File downloaded successfully. Size: $([math]::Round($fileSize/1MB, 2)) MB"
                    
                    Write-Host "Extracting files..."
                    Expand-Archive -Path $zipPath -DestinationPath $downloadDir -Force
                    Write-Host "Extraction completed!"

                    Remove-Item -Path $zipPath
                    Write-Host "Cleaned up temporary ZIP file"

                    Write-Host "Intel RealSense SDK v2.56.3 has been downloaded and extracted to: $downloadDir"
                    Write-Host "You can find the source code in the librealsense-2.56.3 subdirectory"
                }
                else {
                    throw "Downloaded file is empty"
                }
            }
            else {
                throw "File download failed"
            }
        }
        catch {
            Write-Error "An error occurred: $_"
            Write-Host "Exception type: $($_.Exception.GetType().FullName)"
            Write-Host "Exception message: $($_.Exception.Message)"
        }
    }
    else {
        Write-Output "Directory already exists at $completeDir where RealSense source files exists. Using it as CMake target for building librealsense2 package."
    }
    
    Set-Location $completeDir
    if (Test-Path -Path ./build) {
        Write-Host "Cleaning existing build directory..."
        Remove-Item -Path ./build -Recurse -Force
    }
    New-Item -ItemType Directory -Path ./build -Force
    Set-Location ./build
    cmake .. `
        -DCMAKE_BUILD_TYPE=Release `
        -DCMAKE_INSTALL_PREFIX="../../librealsense-2.56.3/install" `
        -DBUILD_EXAMPLES=ON `
        -DBUILD_GRAPHICAL_EXAMPLES=OFF `
        -DBUILD_WITH_CUDA=OFF `
        -DBUILD_NETWORK_DEVICE=OFF `
        -DBUILD_WITH_STATIC_CRT=OFF `
        -DBUILD_UNIT_TESTS=ON `
        -DBUILD_TOOLS=ON
    cmake --build . --config Release --clean-first
    cmake --build . --config Release
    cmake --install . --config Release
}
catch {
    Write-Error "An error occurred: $_"
    Write-Host "Exception type: $($_.Exception.GetType().FullName)"
    Write-Host "Exception message: $($_.Exception.Message)"
}
