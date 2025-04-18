#!/bin/bash

sourcePath="librealsense-2.56.3"
sdkUrl="https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.56.3.zip"

downloadDir="../realsense-source"
zipPath="$downloadDir/realsense-sdk-v2.56.3.zip"
completeDir="$downloadDir/$sourcePath"

if [ ! -d "$completeDir" ]; then
    try_download() {
        if [ ! -d "$downloadDir" ]; then
            mkdir -p "$downloadDir"
            echo "Created directory: $downloadDir"
        fi

        echo "Downloading Intel RealSense SDK v2.56.3..."
        
        # Download the file
        if ! curl -L --silent --show-error --fail "$sdkUrl" -o "$zipPath"; then
            echo "Error: Failed to download the file" >&2
            return 1
        fi
        
        echo "Download completed successfully!"

        if [ -f "$zipPath" ]; then
            fileSize=$(stat -c %s "$zipPath" 2>/dev/null || stat -f %z "$zipPath")
            fileSizeMB=$(echo "scale=2; $fileSize/1048576" | bc)
            
            if [ "$fileSize" -gt 0 ]; then
                echo "File downloaded successfully. Size: $fileSizeMB MB"
                
                echo "Extracting files..."
                unzip -q "$zipPath" -d "$downloadDir"
                if [ $? -ne 0 ]; then
                    echo "Error: Extraction failed" >&2
                    return 1
                fi
                echo "Extraction completed!"

                rm -f "$zipPath"
                echo "Cleaned up temporary ZIP file"

                echo "Intel RealSense SDK v2.56.3 has been downloaded and extracted to: $downloadDir"
                echo "You can find the source code in the librealsense-2.56.3 subdirectory"
                return 0
            else
                echo "Error: Downloaded file is empty" >&2
                return 1
            fi
        else
            echo "Error: File download failed" >&2
            return 1
        fi
    }

    # Try the download and handle errors
    if ! try_download; then
        echo "An error occurred during download or extraction" >&2
        exit 1
    fi
else
    echo "Directory already exists at $completeDir where RealSense source files exists. Using it as CMake target for building librealsense2 package."
fi