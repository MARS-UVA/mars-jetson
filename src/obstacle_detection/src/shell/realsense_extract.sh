#!/bin/bash

SOURCE_PATH="librealsense-2.56.3"
SDK_URL="https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.56.3.zip"

DOWNLOAD_DIR="../realsense-source"
ZIP_PATH="${DOWNLOAD_DIR}/realsense-sdk-v2.56.3.zip"
COMPLETE_DIR="${DOWNLOAD_DIR}/${SOURCE_PATH}"

if [ ! -d "${COMPLETE_DIR}" ]; then
    try_download() {
        if [ ! -d "${DOWNLOAD_DIR}" ]; then
            mkdir -p "${DOWNLOAD_DIR}"
            echo "Created directory: ${DOWNLOAD_DIR}"
        fi

        echo "Downloading Intel RealSense SDK v2.56.3..."
        
        if wget -q --show-progress --progress=bar:force "${SDK_URL}" -O "${ZIP_PATH}"; then
            echo "Download completed successfully!"
            
            if [ -f "${ZIP_PATH}" ]; then
                FILE_SIZE=$(du -h "${ZIP_PATH}" | cut -f1)
                echo "File downloaded successfully. Size: ${FILE_SIZE}"
                
                echo "Extracting files..."
                unzip -q "${ZIP_PATH}" -d "${DOWNLOAD_DIR}"
                echo "Extraction completed!"
                
                rm "${ZIP_PATH}"
                echo "Cleaned up temporary ZIP file"
                
                echo "Intel RealSense SDK v2.56.3 has been downloaded and extracted to: ${DOWNLOAD_DIR}"
                echo "You can find the source code in the librealsense-2.56.3 subdirectory"
                return 0
            else
                echo "File download failed"
                return 1
            fi
        else
            echo "Download failed"
            return 1
        fi
    }
    
    if ! try_download; then
        echo "An error occurred during download or extraction"
        exit 1
    fi
else
    echo "Directory already exists at ${COMPLETE_DIR} where RealSense source files exists. Using it as CMake target for building librealsense2 package."
fi